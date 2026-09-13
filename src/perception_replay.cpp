#include "robotnav/perception/replay.h"

#include <charconv>
#include <cmath>
#include <iomanip>
#include <istream>
#include <limits>
#include <ostream>
#include <sstream>
#include <string_view>
#include <system_error>
#include <utility>

namespace robotnav::perception {
namespace {

using Record = std::vector<std::string>;

ReplayResult failure(std::size_t line, std::string message,
                     std::size_t frame_count = 0) {
    return {false, frame_count, line, std::move(message)};
}

std::string precise(double value) {
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << value;
    return output.str();
}

std::string csvField(const std::string& value) {
    if (value.find_first_of(",\"\r\n") == std::string::npos) return value;
    std::string result = "\"";
    for (char character : value) {
        if (character == '"') result.push_back('"');
        result.push_back(character);
    }
    result.push_back('"');
    return result;
}

std::string jsonString(const std::string& value) {
    static constexpr char kHex[] = "0123456789abcdef";
    std::string result = "\"";
    for (const unsigned char character : value) {
        switch (character) {
            case '"': result += "\\\""; break;
            case '\\': result += "\\\\"; break;
            case '\b': result += "\\b"; break;
            case '\f': result += "\\f"; break;
            case '\n': result += "\\n"; break;
            case '\r': result += "\\r"; break;
            case '\t': result += "\\t"; break;
            default:
                if (character < 0x20) {
                    result += "\\u00";
                    result.push_back(kHex[character >> 4]);
                    result.push_back(kHex[character & 0x0f]);
                } else {
                    result.push_back(static_cast<char>(character));
                }
        }
    }
    result.push_back('"');
    return result;
}

bool writeRecord(std::ostream& output, const Record& record,
                 ReplayFormat format) {
    if (format == ReplayFormat::Csv) {
        for (std::size_t index = 0; index < record.size(); ++index) {
            if (index != 0) output.put(',');
            output << csvField(record[index]);
        }
    } else {
        output.put('[');
        for (std::size_t index = 0; index < record.size(); ++index) {
            if (index != 0) output.put(',');
            output << jsonString(record[index]);
        }
        output.put(']');
    }
    output.put('\n');
    return output.good();
}

enum class CsvParseStatus { Ok, Incomplete, Malformed };

CsvParseStatus parseCsv(const std::string& line, Record& record) {
    record.clear();
    std::string field;
    bool quoted = false;
    bool quote_closed = false;
    for (std::size_t index = 0; index < line.size(); ++index) {
        const char character = line[index];
        if (quoted) {
            if (character == '"') {
                if (index + 1 < line.size() && line[index + 1] == '"') {
                    field.push_back('"');
                    ++index;
                } else {
                    quoted = false;
                    quote_closed = true;
                }
            } else {
                field.push_back(character);
            }
            continue;
        }
        if (quote_closed && character != ',') {
            return CsvParseStatus::Malformed;
        }
        if (character == ',') {
            record.push_back(std::move(field));
            field.clear();
            quote_closed = false;
        } else if (character == '"' && field.empty()) {
            quoted = true;
        } else {
            field.push_back(character);
        }
    }
    if (quoted) return CsvParseStatus::Incomplete;
    record.push_back(std::move(field));
    return CsvParseStatus::Ok;
}

int hexValue(char character) {
    if (character >= '0' && character <= '9') return character - '0';
    if (character >= 'a' && character <= 'f') return character - 'a' + 10;
    if (character >= 'A' && character <= 'F') return character - 'A' + 10;
    return -1;
}

void skipSpace(std::string_view line, std::size_t& index) {
    while (index < line.size() &&
           (line[index] == ' ' || line[index] == '\t' ||
            line[index] == '\r')) {
        ++index;
    }
}

bool parseJsonString(std::string_view line, std::size_t& index,
                     std::string& value) {
    if (index >= line.size() || line[index++] != '"') return false;
    value.clear();
    while (index < line.size()) {
        const unsigned char character = line[index++];
        if (character == '"') return true;
        if (character < 0x20) return false;
        if (character != '\\') {
            value.push_back(static_cast<char>(character));
            continue;
        }
        if (index >= line.size()) return false;
        const char escape = line[index++];
        switch (escape) {
            case '"': value.push_back('"'); break;
            case '\\': value.push_back('\\'); break;
            case '/': value.push_back('/'); break;
            case 'b': value.push_back('\b'); break;
            case 'f': value.push_back('\f'); break;
            case 'n': value.push_back('\n'); break;
            case 'r': value.push_back('\r'); break;
            case 't': value.push_back('\t'); break;
            case 'u': {
                if (index + 4 > line.size()) return false;
                int codepoint = 0;
                for (int digit = 0; digit < 4; ++digit) {
                    const int value_digit = hexValue(line[index++]);
                    if (value_digit < 0) return false;
                    codepoint = codepoint * 16 + value_digit;
                }
                if (codepoint > 0x7f) return false;
                value.push_back(static_cast<char>(codepoint));
                break;
            }
            default: return false;
        }
    }
    return false;
}

bool parseJsonLinesRecord(const std::string& line, Record& record) {
    record.clear();
    std::size_t index = 0;
    skipSpace(line, index);
    if (index >= line.size() || line[index++] != '[') return false;
    skipSpace(line, index);
    if (index < line.size() && line[index] == ']') {
        ++index;
        skipSpace(line, index);
        return index == line.size();
    }
    while (index < line.size()) {
        std::string value;
        if (!parseJsonString(line, index, value)) return false;
        record.push_back(std::move(value));
        skipSpace(line, index);
        if (index >= line.size()) return false;
        if (line[index] == ']') {
            ++index;
            skipSpace(line, index);
            return index == line.size();
        }
        if (line[index++] != ',') return false;
        skipSpace(line, index);
    }
    return false;
}

bool readRecord(std::istream& input, ReplayFormat format,
                std::size_t& line_number, Record& record) {
    std::string line;
    if (!std::getline(input, line)) return false;
    ++line_number;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (format == ReplayFormat::JsonLines) {
        return parseJsonLinesRecord(line, record);
    }
    auto csv_status = parseCsv(line, record);
    while (csv_status == CsvParseStatus::Incomplete) {
        std::string continuation;
        if (!std::getline(input, continuation)) return false;
        ++line_number;
        if (!continuation.empty() && continuation.back() == '\r') {
            continuation.pop_back();
        }
        line.push_back('\n');
        line += continuation;
        csv_status = parseCsv(line, record);
    }
    return csv_status == CsvParseStatus::Ok;
}

template <typename Integer>
bool parseInteger(const std::string& text, Integer& value) {
    if (text.empty()) return false;
    const char* begin = text.data();
    const char* end = begin + text.size();
    const auto result = std::from_chars(begin, end, value);
    return result.ec == std::errc{} && result.ptr == end;
}

bool parseDouble(const std::string& text, double& value) {
    if (text.empty()) return false;
    std::size_t parsed = 0;
    try {
        value = std::stod(text, &parsed);
    } catch (...) {
        return false;
    }
    return parsed == text.size();
}

bool parseBool(const std::string& text, bool& value) {
    if (text == "1" || text == "true") {
        value = true;
        return true;
    }
    if (text == "0" || text == "false") {
        value = false;
        return true;
    }
    return false;
}

bool parseObservationKind(const std::string& text, ObservationKind& kind) {
    if (text == "unknown") kind = ObservationKind::Unknown;
    else if (text == "static") kind = ObservationKind::StaticObstacle;
    else if (text == "dynamic") kind = ObservationKind::DynamicObstacle;
    else return false;
    return true;
}

bool parseTrackState(const std::string& text, TrackState& state) {
    if (text == "tentative") state = TrackState::Tentative;
    else if (text == "confirmed") state = TrackState::Confirmed;
    else if (text == "coasting") state = TrackState::Coasting;
    else return false;
    return true;
}

Record sensorFrameRecord(const SensorFrame& frame) {
    return {"sensor_frame", std::to_string(frame.schema_version),
            std::to_string(frame.sequence), std::to_string(frame.timestamp_ns),
            frame.frame_id, frame.source_id, precise(frame.sensor_pose.x),
            precise(frame.sensor_pose.y), precise(frame.sensor_pose.theta),
            std::to_string(frame.points.size())};
}

Record sensorPointRecord(const SensorPoint& point) {
    return {"sensor_point", precise(point.position.x),
            precise(point.position.y), point.hit ? "1" : "0",
            std::string(toString(point.kind)), precise(point.covariance.xx),
            precise(point.covariance.xy), precise(point.covariance.yy)};
}

Record trackFrameRecord(const ObstacleTrackFrame& frame) {
    return {"track_frame", std::to_string(frame.schema_version),
            std::to_string(frame.sequence), std::to_string(frame.timestamp_ns),
            frame.frame_id, frame.source_id,
            std::to_string(frame.tracks.size())};
}

Record trackRecord(const ObstacleTrack& track) {
    return {"track", std::to_string(track.track_id),
            std::to_string(track.timestamp_ns),
            std::to_string(track.last_observation_timestamp_ns),
            track.frame_id, precise(track.position.x),
            precise(track.position.y), precise(track.velocity.x),
            precise(track.velocity.y), precise(track.acceleration.x),
            precise(track.acceleration.y), precise(track.radius),
            precise(track.position_covariance.xx),
            precise(track.position_covariance.xy),
            precise(track.position_covariance.yy), precise(track.confidence),
            std::to_string(track.age), std::to_string(track.hits),
            std::to_string(track.missed_observations),
            std::string(toString(track.state))};
}

bool parseSensorHeader(const Record& record, SensorFrame& frame,
                       std::size_t& point_count) {
    return record.size() == 10 && record[0] == "sensor_frame" &&
        parseInteger(record[1], frame.schema_version) &&
        parseInteger(record[2], frame.sequence) &&
        parseInteger(record[3], frame.timestamp_ns) &&
        (frame.frame_id = record[4], true) &&
        (frame.source_id = record[5], true) &&
        parseDouble(record[6], frame.sensor_pose.x) &&
        parseDouble(record[7], frame.sensor_pose.y) &&
        parseDouble(record[8], frame.sensor_pose.theta) &&
        parseInteger(record[9], point_count);
}

bool parseSensorPoint(const Record& record, SensorPoint& point) {
    return record.size() == 8 && record[0] == "sensor_point" &&
        parseDouble(record[1], point.position.x) &&
        parseDouble(record[2], point.position.y) &&
        parseBool(record[3], point.hit) &&
        parseObservationKind(record[4], point.kind) &&
        parseDouble(record[5], point.covariance.xx) &&
        parseDouble(record[6], point.covariance.xy) &&
        parseDouble(record[7], point.covariance.yy);
}

bool parseTrackHeader(const Record& record, ObstacleTrackFrame& frame,
                      std::size_t& track_count) {
    return record.size() == 7 && record[0] == "track_frame" &&
        parseInteger(record[1], frame.schema_version) &&
        parseInteger(record[2], frame.sequence) &&
        parseInteger(record[3], frame.timestamp_ns) &&
        (frame.frame_id = record[4], true) &&
        (frame.source_id = record[5], true) &&
        parseInteger(record[6], track_count);
}

bool parseTrack(const Record& record, ObstacleTrack& track) {
    return record.size() == 20 && record[0] == "track" &&
        parseInteger(record[1], track.track_id) &&
        parseInteger(record[2], track.timestamp_ns) &&
        parseInteger(record[3], track.last_observation_timestamp_ns) &&
        (track.frame_id = record[4], true) &&
        parseDouble(record[5], track.position.x) &&
        parseDouble(record[6], track.position.y) &&
        parseDouble(record[7], track.velocity.x) &&
        parseDouble(record[8], track.velocity.y) &&
        parseDouble(record[9], track.acceleration.x) &&
        parseDouble(record[10], track.acceleration.y) &&
        parseDouble(record[11], track.radius) &&
        parseDouble(record[12], track.position_covariance.xx) &&
        parseDouble(record[13], track.position_covariance.xy) &&
        parseDouble(record[14], track.position_covariance.yy) &&
        parseDouble(record[15], track.confidence) &&
        parseInteger(record[16], track.age) &&
        parseInteger(record[17], track.hits) &&
        parseInteger(record[18], track.missed_observations) &&
        parseTrackState(record[19], track.state);
}

ReplayResult writePreamble(std::ostream& output, ReplayFormat format,
                           const std::string& payload) {
    if (!writeRecord(output,
                     {"robotnav_perception_replay", "1", payload}, format)) {
        return failure(0, "failed to write replay preamble");
    }
    return {};
}

ReplayResult readPreamble(std::istream& input, ReplayFormat format,
                          const std::string& payload,
                          std::size_t& line_number) {
    Record record;
    const auto previous_line = line_number;
    if (!readRecord(input, format, line_number, record)) {
        return failure(line_number == previous_line
                           ? line_number + 1 : line_number,
                       "missing or malformed replay preamble");
    }
    if (record != Record{"robotnav_perception_replay", "1", payload}) {
        return failure(line_number, "unsupported replay preamble");
    }
    return {};
}

template <typename Frame>
ReplayResult validateFrames(const std::vector<Frame>& frames) {
    for (std::size_t index = 0; index < frames.size(); ++index) {
        const auto validation = validate(frames[index]);
        if (!validation) {
            return failure(index + 1,
                           "invalid frame: " +
                               std::string(toString(validation.error)));
        }
    }
    return {};
}

}  // namespace

ReplayResult writeSensorFrames(std::ostream& output,
                               const std::vector<SensorFrame>& frames,
                               ReplayFormat format) {
    const auto input_validation = validateFrames(frames);
    if (!input_validation) return input_validation;
    auto result = writePreamble(output, format, "sensor_frames");
    if (!result) return result;
    for (const auto& frame : frames) {
        if (!writeRecord(output, sensorFrameRecord(frame), format)) {
            return failure(0, "failed to write sensor frame",
                           result.frame_count);
        }
        for (const auto& point : frame.points) {
            if (!writeRecord(output, sensorPointRecord(point), format)) {
                return failure(0, "failed to write sensor point",
                               result.frame_count);
            }
        }
        ++result.frame_count;
    }
    return result;
}

ReplayResult writeObstacleTrackFrames(
    std::ostream& output,
    const std::vector<ObstacleTrackFrame>& frames,
    ReplayFormat format) {
    const auto input_validation = validateFrames(frames);
    if (!input_validation) return input_validation;
    auto result = writePreamble(output, format, "track_frames");
    if (!result) return result;
    for (const auto& frame : frames) {
        if (!writeRecord(output, trackFrameRecord(frame), format)) {
            return failure(0, "failed to write track frame",
                           result.frame_count);
        }
        for (const auto& track : frame.tracks) {
            if (!writeRecord(output, trackRecord(track), format)) {
                return failure(0, "failed to write track",
                               result.frame_count);
            }
        }
        ++result.frame_count;
    }
    return result;
}

ReplayResult replaySensorFrames(std::istream& input, ReplayFormat format,
                                const SensorFrameCallback& callback) {
    if (!callback) return failure(0, "sensor frame callback is empty");
    std::size_t line_number = 0;
    auto result = readPreamble(input, format, "sensor_frames", line_number);
    if (!result) return result;
    while (input.peek() != std::char_traits<char>::eof()) {
        Record record;
        if (!readRecord(input, format, line_number, record)) {
            return failure(line_number, "malformed sensor frame record",
                           result.frame_count);
        }
        SensorFrame frame;
        std::size_t point_count = 0;
        if (!parseSensorHeader(record, frame, point_count)) {
            return failure(line_number, "invalid sensor frame header",
                           result.frame_count);
        }
        for (std::size_t point_index = 0;
             point_index < point_count; ++point_index) {
            const auto previous_line = line_number;
            if (!readRecord(input, format, line_number, record)) {
                return failure(line_number == previous_line
                                   ? line_number + 1 : line_number,
                               "truncated or malformed sensor frame",
                               result.frame_count);
            }
            SensorPoint point;
            if (!parseSensorPoint(record, point)) {
                return failure(line_number, "invalid sensor point",
                               result.frame_count);
            }
            frame.points.push_back(point);
        }
        const auto validation = validate(frame);
        if (!validation) {
            return failure(line_number,
                           "invalid sensor frame: " +
                               std::string(toString(validation.error)),
                           result.frame_count);
        }
        callback(frame);
        ++result.frame_count;
    }
    return result;
}

ReplayResult replayObstacleTrackFrames(
    std::istream& input, ReplayFormat format,
    const ObstacleTrackFrameCallback& callback) {
    if (!callback) return failure(0, "track frame callback is empty");
    std::size_t line_number = 0;
    auto result = readPreamble(input, format, "track_frames", line_number);
    if (!result) return result;
    while (input.peek() != std::char_traits<char>::eof()) {
        Record record;
        if (!readRecord(input, format, line_number, record)) {
            return failure(line_number, "malformed track frame record",
                           result.frame_count);
        }
        ObstacleTrackFrame frame;
        std::size_t track_count = 0;
        if (!parseTrackHeader(record, frame, track_count)) {
            return failure(line_number, "invalid track frame header",
                           result.frame_count);
        }
        for (std::size_t track_index = 0;
             track_index < track_count; ++track_index) {
            const auto previous_line = line_number;
            if (!readRecord(input, format, line_number, record)) {
                return failure(line_number == previous_line
                                   ? line_number + 1 : line_number,
                               "truncated or malformed track frame",
                               result.frame_count);
            }
            ObstacleTrack track;
            if (!parseTrack(record, track)) {
                return failure(line_number, "invalid obstacle track",
                               result.frame_count);
            }
            frame.tracks.push_back(track);
        }
        const auto validation = validate(frame);
        if (!validation) {
            return failure(line_number,
                           "invalid track frame: " +
                               std::string(toString(validation.error)),
                           result.frame_count);
        }
        callback(frame);
        ++result.frame_count;
    }
    return result;
}

}  // namespace robotnav::perception
