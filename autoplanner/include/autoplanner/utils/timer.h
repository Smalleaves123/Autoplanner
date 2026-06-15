#pragma once

#include <chrono>
#include <string>

namespace autoplanner {

// Simple RAII timer for measuring wall-clock elapsed time.
// Usage:
//   Timer t;
//   // ... work ...
//   double ms = t.elapsedMs();
class Timer {
public:
    using Clock = std::chrono::steady_clock;

    Timer() : start_(Clock::now()) {}

    // Reset the start point to now.
    void reset() { start_ = Clock::now(); }

    // Elapsed time in milliseconds (double).
    double elapsedMs() const {
        auto end = Clock::now();
        return std::chrono::duration<double, std::milli>(end - start_).count();
    }

    // Elapsed time in microseconds.
    double elapsedUs() const {
        auto end = Clock::now();
        return std::chrono::duration<double, std::micro>(end - start_).count();
    }

private:
    Clock::time_point start_;
};

}  // namespace autoplanner
