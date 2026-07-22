#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "autompc/core/types.h"

namespace robotnav {

struct TraceSample {
    double time = 0.0;
    autompc::State state;
    autompc::Control command;
    double cross_track_error = 0.0;
    double heading_error = 0.0;
};

class NavigationTrace {
public:
    void append(const TraceSample& sample) { samples_.push_back(sample); }

    const std::vector<TraceSample>& samples() const { return samples_; }
    std::size_t size() const { return samples_.size(); }
    bool empty() const { return samples_.empty(); }

private:
    std::vector<TraceSample> samples_;
};

bool saveNavigationTraceCsv(const NavigationTrace& trace,
                            const std::string& file_path);

}  // namespace robotnav
