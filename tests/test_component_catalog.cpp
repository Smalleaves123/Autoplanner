#include <gtest/gtest.h>

#include <algorithm>

#include "robotnav/component_catalog.h"

using namespace robotnav;

TEST(ComponentCatalog, ListsAllPipelineExtensionPoints) {
    const auto catalog = availableComponents();
    EXPECT_NE(std::find(catalog.planners.begin(), catalog.planners.end(),
                        "astar"), catalog.planners.end());
    EXPECT_NE(std::find(catalog.planners.begin(), catalog.planners.end(),
                        "space_time_astar"), catalog.planners.end());
    EXPECT_NE(std::find(catalog.controllers.begin(), catalog.controllers.end(),
                        "stanley"), catalog.controllers.end());
    EXPECT_NE(std::find(catalog.local_planners.begin(),
                        catalog.local_planners.end(), "none"),
              catalog.local_planners.end());
    EXPECT_NE(std::find(catalog.smoothers.begin(), catalog.smoothers.end(),
                        "curvature"), catalog.smoothers.end());
}

TEST(ComponentCatalog, ValidatesStaticAndDynamicSelections) {
    PipelineConfig config;
    EXPECT_TRUE(validateComponentSelection(config).valid);

    config.planner = "space_time_astar";
    EXPECT_FALSE(validateComponentSelection(config).valid);
    EXPECT_TRUE(validateComponentSelection(config, true).valid);

    config.planner = "astar";
    config.local_planner = "missing";
    const auto invalid = validateComponentSelection(config);
    EXPECT_FALSE(invalid.valid);
    EXPECT_NE(invalid.message.find("local planner"), std::string::npos);
}
