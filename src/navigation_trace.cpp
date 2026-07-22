#include "robotnav/navigation_trace.h"

#include <fstream>
#include <iomanip>

namespace robotnav {

bool saveNavigationTraceCsv(const NavigationTrace& trace,
                            const std::string& file_path) {
    std::ofstream output(file_path);
    if (!output.is_open()) return false;

    output << "time,x,y,theta,velocity,command_velocity,command_steering,"
              "cross_track_error,heading_error\n";
    output << std::fixed << std::setprecision(8);
    for (const auto& sample : trace.samples()) {
        output << sample.time << ','
               << sample.state.x << ','
               << sample.state.y << ','
               << sample.state.theta << ','
               << sample.state.v << ','
               << sample.command.velocity << ','
               << sample.command.steering << ','
               << sample.cross_track_error << ','
               << sample.heading_error << '\n';
    }
    return true;
}

}  // namespace robotnav
