#pragma once

#include <filesystem>
#include <string>

namespace robotnav_test {

inline std::filesystem::path artifactPath(const std::string& filename) {
    const auto directory = std::filesystem::path("test_artifacts") /
                           "robotnav";
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    return directory / filename;
}

struct ArtifactCleanup {
    ~ArtifactCleanup() {
        std::error_code error;
        std::filesystem::remove_all(
            std::filesystem::path("test_artifacts") / "robotnav", error);
    }
};

inline const ArtifactCleanup artifact_cleanup{};

}  // namespace robotnav_test
