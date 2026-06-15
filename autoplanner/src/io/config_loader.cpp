#include "autoplanner/io/config_loader.h"

#include <cctype>
#include <fstream>
#include <sstream>

namespace autoplanner {
namespace io {

bool ConfigLoader::load(const std::string& path) {
    std::ifstream fin(path);
    if (!fin.is_open()) return false;

    std::string line;
    while (std::getline(fin, line)) {
        // Trim whitespace
        size_t start = 0;
        while (start < line.size() && std::isspace(static_cast<unsigned char>(line[start])))
            start++;
        size_t end = line.size();
        while (end > start && std::isspace(static_cast<unsigned char>(line[end - 1])))
            end--;
        line = line.substr(start, end - start);

        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') continue;

        // Skip section headers
        if (line[0] == '[') continue;

        // Parse key = value
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key = line.substr(0, eq);
        std::string value = line.substr(eq + 1);

        // Trim key
        size_t kend = key.size();
        while (kend > 0 && std::isspace(static_cast<unsigned char>(key[kend - 1])))
            kend--;
        key = key.substr(0, kend);

        // Trim value
        size_t vstart = 0;
        while (vstart < value.size() && std::isspace(static_cast<unsigned char>(value[vstart])))
            vstart++;
        value = value.substr(vstart);

        // Remove quotes if present
        if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
            value = value.substr(1, value.size() - 2);

        values_[key] = value;
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
    return v == "true" || v == "1" || v == "yes" || v == "on";
}

bool ConfigLoader::hasKey(const std::string& key) const {
    return values_.find(key) != values_.end();
}

}  // namespace io
}  // namespace autoplanner
