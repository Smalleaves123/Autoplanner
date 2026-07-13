#pragma once

#include "autoplanner/collision/collision_checker.h"
#include "autoplanner/core/grid_map.h"

namespace autoplanner {

enum class FootprintType {
    Circle,
    Rectangle,
};

struct RobotFootprint {
    FootprintType type = FootprintType::Circle;
    double radius = 0.0;
    double length = 0.0;
    double width = 0.0;

    static RobotFootprint circle(double radius);
    static RobotFootprint rectangle(double length, double width);
};

// Collision checker that evaluates a finite-size robot against the occupancy
// grid. Rectangle footprints are rotated using the pose heading.
class FootprintCollisionChecker final : public CollisionChecker {
public:
    FootprintCollisionChecker(const GridMap& map, RobotFootprint footprint);

    bool isStateValid(const Point2d& p) const override;
    bool isSegmentValid(const Point2d& p1, const Point2d& p2) const override;
    bool isPathValid(const std::vector<Point2d>& path) const override;

    bool isPoseValid(const Pose2d& pose) const override;
    bool isPoseSegmentValid(const Pose2d& p1,
                            const Pose2d& p2) const override;
    bool isPosePathValid(const std::vector<Pose2d>& path) const override;

    const RobotFootprint& footprint() const { return footprint_; }

private:
    const GridMap& map_;
    RobotFootprint footprint_;
};

}  // namespace autoplanner
