#include "autoplanner/io/map_saver.h"

#include <fstream>

namespace autoplanner {
namespace io {

bool saveMap(const GridMap& map, const std::string& path) {
    std::ofstream fout(path);
    if (!fout.is_open()) return false;

    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            fout << (map.isOccupied(x, y) ? '1' : '0');
        }
        fout << '\n';
    }
    return true;
}

}  // namespace io
}  // namespace autoplanner
