#pragma once

#include <iostream>
#include <sstream>
#include <string>

namespace autoplanner {

// Minimal logging utility with configurable levels.
//
// Usage:
//   Logger::setLevel(Logger::Level::Debug);
//   LOG_INFO("Planning started on map " << map.width() << "x" << map.height());
//
// By default, only WARN and ERROR are printed.  Set the level at startup.
class Logger {
public:
    enum class Level { Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4 };

    static void setLevel(Level level) { min_level_ = level; }
    static Level level() { return min_level_; }

    static void debug(const std::string& msg)   { log(Level::Debug, "DEBUG", msg); }
    static void info(const std::string& msg)    { log(Level::Info,  "INFO ", msg); }
    static void warn(const std::string& msg)    { log(Level::Warn,  "WARN ", msg); }
    static void error(const std::string& msg)   { log(Level::Error, "ERROR", msg); }

private:
    static inline Level min_level_ = Level::Warn;

    static void log(Level lvl, const char* tag, const std::string& msg) {
        if (lvl < min_level_) return;
        std::cerr << "[" << tag << "] " << msg << "\n";
    }
};

// Stream-based macros for convenience.
#define LOG_DEBUG(expr) do { \
    std::ostringstream _oss; _oss << expr; \
    autoplanner::Logger::debug(_oss.str()); \
} while(0)

#define LOG_INFO(expr) do { \
    std::ostringstream _oss; _oss << expr; \
    autoplanner::Logger::info(_oss.str()); \
} while(0)

#define LOG_WARN(expr) do { \
    std::ostringstream _oss; _oss << expr; \
    autoplanner::Logger::warn(_oss.str()); \
} while(0)

#define LOG_ERROR(expr) do { \
    std::ostringstream _oss; _oss << expr; \
    autoplanner::Logger::error(_oss.str()); \
} while(0)

}  // namespace autoplanner
