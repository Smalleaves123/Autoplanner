#include <gtest/gtest.h>
#include <fstream>

#include "autoplanner/io/config_loader.h"

using namespace autoplanner::io;

TEST(ConfigLoader, LoadFile) {
    // Write a temporary config file
    std::ofstream fout("/tmp/test_config.txt");
    fout << "# test config\n";
    fout << "planner = astar\n";
    fout << "resolution = 0.05\n";
    fout << "max_iter = 5000\n";
    fout << "use_diagonal = true\n";
    fout << "step_size = 2.5\n";
    fout.close();

    ConfigLoader cfg;
    ASSERT_TRUE(cfg.load("/tmp/test_config.txt"));
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
    std::ofstream fout("/tmp/test_empty.txt");
    fout << "# just a comment\n\n";
    fout.close();

    ConfigLoader cfg;
    ASSERT_TRUE(cfg.load("/tmp/test_empty.txt"));
    EXPECT_EQ(cfg.size(), 0u);
}

TEST(ConfigLoader, LoadYamlScalars) {
    std::ofstream fout("/tmp/test_config.yaml");
    fout << "planner: improved_astar\n";
    fout << "map:\n";
    fout << "  resolution: 0.05\n";
    fout << "robot:\n";
    fout << "  radius: 0.25 # metres\n";
    fout << "astar:\n";
    fout << "  allow_diagonal: false\n";
    fout.close();

    ConfigLoader cfg;
    ASSERT_TRUE(cfg.load("/tmp/test_config.yaml"));
    EXPECT_EQ(cfg.getString("planner"), "improved_astar");
    EXPECT_DOUBLE_EQ(cfg.getDouble("map.resolution"), 0.05);
    EXPECT_DOUBLE_EQ(cfg.getDouble("robot.radius"), 0.25);
    EXPECT_FALSE(cfg.getBool("astar.allow_diagonal", true));
}
