#pragma once

#include <string>
#include <unordered_map>

namespace autoplanner {
namespace io {

// Simple INI/YAML-style config loader.
// Reads key=value pairs from a file.  Supports # comments and sections.
//
// File format:
//   # comment
//   key = value
//   [section]
//   key = value
class ConfigLoader {
public:
    // Load configuration from a file.  Returns true on success.
    bool load(const std::string& path);

    // Get a string value.  Returns empty string if key not found.
    std::string getString(const std::string& key,
                          const std::string& default_val = "") const;

    // Get a double value.
    double getDouble(const std::string& key, double default_val = 0.0) const;

    // Get an integer value.
    int getInt(const std::string& key, int default_val = 0) const;

    // Get a boolean value.  Recognises "true"/"1"/"yes".
    bool getBool(const std::string& key, bool default_val = false) const;

    // Check if a key exists.
    bool hasKey(const std::string& key) const;

    // Number of loaded key-value pairs.
    size_t size() const { return values_.size(); }

private:
    std::unordered_map<std::string, std::string> values_;
};

}  // namespace io
}  // namespace autoplanner
