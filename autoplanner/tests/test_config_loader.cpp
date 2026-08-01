#include <gtest/gtest.h>
#include <fstream>

#include "autoplanner/io/config_loader.h"
#include "test_file_utils.h"

using namespace autoplanner::io;

TEST(ConfigLoader, LoadFile) {
    // Write a temporary config file
    const auto path = autoplanner_test::artifactPath("test_config.txt");
    std::ofstream fout(path);
    fout << "# test config\n";
    fout << "planner = astar\n";
    fout << "resolution = 0.05\n";
    fout << "max_iter = 5000\n";
    fout << "use_diagonal = true\n";
    fout << "step_size = 2.5\n";
    fout.close();

    ConfigLoader cfg;
    ASSERT_TRUE(cfg.load(path.string()));
    EXPECT_EQ(cfg.size(), 5u);

    EXPECT_EQ(cfg.getString("planner"), "astar");
    EXPECT_DOUBLE_EQ(cfg.getDouble("resolution"), 0.05);
    EXPECT_EQ(cfg.getInt("max_iter"), 5000);
    EXPECT_TRUE(cfg.getBool("use_diagonal"));
    EXPECT_DOUBLE_EQ(cfg.getDouble("step_size"), 2.5);
}

TEST(ConfigLoader, MissingKeyReturnsDefault) {
    ConfigLoader cfg;
    EXPECT_EQ(cfg.getString("nope", "fallback"), "fallback");
    EXPECT_DOUBLE_EQ(cfg.getDouble("nope", 3.14), 3.14);
    EXPECT_EQ(cfg.getInt("nope", 42), 42);
    EXPECT_FALSE(cfg.getBool("nope", false));
    EXPECT_FALSE(cfg.hasKey("nope"));
}

TEST(ConfigLoader, EmptyFile) {
    const auto path = autoplanner_test::artifactPath("test_empty.txt");
    std::ofstream fout(path);
    fout << "# just a comment\n\n";
    fout.close();

    ConfigLoader cfg;
    ASSERT_TRUE(cfg.load(path.string()));
    EXPECT_EQ(cfg.size(), 0u);
}

TEST(ConfigLoader, LoadYamlScalars) {
    const auto path = autoplanner_test::artifactPath("test_config.yaml");
    std::ofstream fout(path);
    fout << "planner: improved_astar\n";
    fout << "map:\n";
    fout << "  resolution: 0.05\n";
    fout << "robot:\n";
    fout << "  radius: 0.25 # metres\n";
    fout << "astar:\n";
    fout << "  allow_diagonal: false\n";
    fout.close();

    ConfigLoader cfg;
    ASSERT_TRUE(cfg.load(path.string()));
    EXPECT_EQ(cfg.getString("planner"), "improved_astar");
    EXPECT_DOUBLE_EQ(cfg.getDouble("map.resolution"), 0.05);
    EXPECT_DOUBLE_EQ(cfg.getDouble("robot.radius"), 0.25);
    EXPECT_FALSE(cfg.getBool("astar.allow_diagonal", true));
}
