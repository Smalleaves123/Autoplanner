#include "autoplanner/io/config_loader.h"

#include <cctype>
#include <fstream>
#include <sstream>
#include <vector>

namespace autoplanner {
namespace io {

namespace {

std::string trim(const std::string& value) {
    std::size_t begin = 0;
    while (begin < value.size() &&
           std::isspace(static_cast<unsigned char>(value[begin]))) {
        ++begin;
    }

    std::size_t end = value.size();
    while (end > begin &&
           std::isspace(static_cast<unsigned char>(value[end - 1]))) {
        --end;
    }
    return value.substr(begin, end - begin);
}

std::string removeInlineComment(const std::string& value) {
    const std::size_t comment = value.find('#');
    return trim(comment == std::string::npos ? value
                                             : value.substr(0, comment));
}

}  // namespace

bool ConfigLoader::load(const std::string& path) {
    std::ifstream fin(path);
    if (!fin.is_open()) return false;

    values_.clear();
    std::vector<std::pair<int, std::string>> sections;
    std::string line;
    while (std::getline(fin, line)) {
        const std::size_t first_non_space = line.find_first_not_of(" \t");
        if (first_non_space == std::string::npos) continue;
        const int indent = static_cast<int>(first_non_space);
        line = removeInlineComment(line.substr(first_non_space));

        if (line.empty()) continue;

        // INI-style section headers remain supported.
        if (line.front() == '[' && line.back() == ']') {
            sections.clear();
            sections.emplace_back(indent, trim(line.substr(1, line.size() - 2)));
            continue;
        }

        // YAML lists are not needed for scalar planner parameters.
        if (line.front() == '-') continue;

        const std::size_t eq = line.find('=');
        const std::size_t colon = line.find(':');
        std::size_t separator = eq;
        if (separator == std::string::npos ||
            (colon != std::string::npos && colon < separator)) {
            separator = colon;
        }
        if (separator == std::string::npos) continue;

        const std::string key = trim(line.substr(0, separator));
        const std::string value = removeInlineComment(
            line.substr(separator + 1));
        if (key.empty()) continue;

        while (!sections.empty() && sections.back().first >= indent) {
            sections.pop_back();
        }

        if (value.empty()) {
            sections.emplace_back(indent, key);
            continue;
        }

        std::string full_key;
        for (const auto& section : sections) {
            if (!full_key.empty()) full_key += '.';
            full_key += section.second;
        }
        if (!full_key.empty()) full_key += '.';
        full_key += key;

        std::string normalized_value = value;
        if (normalized_value.size() >= 2 &&
            normalized_value.front() == '"' &&
            normalized_value.back() == '"') {
            normalized_value = normalized_value.substr(
                1, normalized_value.size() - 2);
        }

        values_[full_key] = normalized_value;
    }
    return true;
}

std::string ConfigLoader::getString(const std::string& key,
                                     const std::string& default_val) const {
    auto it = values_.find(key);
    return (it != values_.end()) ? it->second : default_val;
}

double ConfigLoader::getDouble(const std::string& key, double default_val) const {
    auto it = values_.find(key);
    if (it == values_.end()) return default_val;
    try { return std::stod(it->second); }
    catch (...) { return default_val; }
}

int ConfigLoader::getInt(const std::string& key, int default_val) const {
    auto it = values_.find(key);
    if (it == values_.end()) return default_val;
    try { return std::stoi(it->second); }
    catch (...) { return default_val; }
}

bool ConfigLoader::getBool(const std::string& key, bool default_val) const {
    auto it = values_.find(key);
    if (it == values_.end()) return default_val;
    std::string v = it->second;
    for (char& c : v) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return v == "true" || v == "1" || v == "yes" || v == "on";
}

bool ConfigLoader::hasKey(const std::string& key) const {
    return values_.find(key) != values_.end();
}

}  // namespace io
}  // namespace autoplanner
