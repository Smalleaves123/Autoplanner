#pragma once

#include <filesystem>
#include <string>

namespace autoplanner_test {

inline std::filesystem::path artifactPath(const std::string& filename) {
    const auto directory = std::filesystem::path("test_artifacts") /
                           "autoplanner";
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    return directory / filename;
}

struct ArtifactCleanup {
    ~ArtifactCleanup() {
        std::error_code error;
        std::filesystem::remove_all(
            std::filesystem::path("test_artifacts") / "autoplanner",
            error);
    }
};

inline const ArtifactCleanup artifact_cleanup{};

}  // namespace autoplanner_test
