#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>

#include "robotnav/trajectory_controller.h"

using namespace robotnav;

namespace {

class RegisteredTestController final : public TrajectoryController {
public:
    autompc::Control compute(
        const autompc::State&,
        const autompc::Trajectory&,
        const autompc::TrajectoryPoint&) override {
        return {0.25, -0.1};
    }
};

}  // namespace

TEST(ControllerRegistry, CreatesBuiltInControllers) {
    EXPECT_NE(createController("pid"), nullptr);
    EXPECT_NE(createController("pure_pursuit"), nullptr);
    EXPECT_NE(createController("stanley"), nullptr);
}

TEST(ControllerRegistry, ListsControllersInDeterministicOrder) {
    const auto names = availableControllers();
    EXPECT_TRUE(std::is_sorted(names.begin(), names.end()));
    EXPECT_NE(std::find(names.begin(), names.end(), "pid"), names.end());
    EXPECT_NE(std::find(names.begin(), names.end(), "stanley"), names.end());
}

TEST(ControllerRegistry, RegistersApplicationController) {
    auto& registry = ControllerRegistry::instance();
    const std::string name = "registered_test";
    registry.unregisterController(name);

    EXPECT_TRUE(registry.registerController(
        name, [](const autompc::SimulationOptions&) {
            return std::make_unique<RegisteredTestController>();
        }));
    auto controller = createController(name);
    ASSERT_NE(controller, nullptr);
    const auto command = controller->compute({}, {}, {});
    EXPECT_DOUBLE_EQ(command.velocity, 0.25);
    EXPECT_DOUBLE_EQ(command.steering, -0.1);

    EXPECT_TRUE(registry.unregisterController(name));
    EXPECT_EQ(createController(name), nullptr);
}
