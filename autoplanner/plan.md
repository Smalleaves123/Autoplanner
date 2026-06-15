
AutoPlanner 项目完整规划文档

## 1. 项目名称

**AutoPlanner：面向移动机器人的 C++ 路径规划与运动规划算法库**

---

## 2. 项目定位

本项目的目标不是写几个零散的算法 demo，而是做成一个完整、可扩展、可测试、可展示的机器人路径规划算法库。

核心定位：

```text
C++ 负责：
- 路径规划核心算法
- 运动规划算法
- 代价地图
- 碰撞检测
- 路径平滑
- 指标统计
- benchmark
- 工程接口

Python 负责：
- 地图生成
- 批量实验
- 数据分析
- 可视化
- benchmark 图表
- 实验报告辅助生成

后期可选：
- ROS2 接口
- RViz 可视化
- 与 MPC / TD-MPC2 / MBRL 方向衔接
````

最终项目应具备以下特点：

```text
1. 不是单个 A* demo，而是一个完整 C++ 算法库。
2. 支持多种路径规划和运动规划算法。
3. 有统一 Planner 接口，便于扩展新算法。
4. 有地图读取、障碍物膨胀、代价地图、碰撞检测、路径平滑等机器人规划常用模块。
5. 有 Python 可视化和批量实验工具。
6. 有 benchmark 和实验报告。
7. 有单元测试和清晰工程结构。
8. 可以写进简历，也可以放到 GitHub 展示。
```

---

## 3. 项目最终目标

给定二维栅格地图、起点、终点、机器人半径和规划算法配置，系统能够输出一条安全、较短、较平滑、可评估的路径。

### 3.1 输入

```text
1. 地图文件
   - txt 栅格地图
   - png 地图，可选

2. 起点
   - start_x
   - start_y

3. 终点
   - goal_x
   - goal_y

4. 机器人参数
   - robot_radius
   - resolution
   - inflation_radius

5. 算法参数
   - planner type
   - heuristic type
   - weight
   - max_iterations
   - step_size
   - goal_sample_rate

6. 输出配置
   - 是否保存 path
   - 是否保存 metrics
   - 是否保存 visualization
```

### 3.2 输出

```text
1. path.csv
2. path.json
3. metrics.json
4. visualization.png
5. expanded_nodes.csv
6. benchmark.csv
7. benchmark_summary.json
8. benchmark 对比图
9. 可选 gif 动图
```

### 3.3 关键指标

```text
success
planning_time_ms
path_length
expanded_nodes
iterations
turning_count
smoothness_score
average_curvature
minimum_obstacle_distance
collision_free
memory_usage_mb
```

---

## 4. 技术栈规划

### 4.1 C++ 技术栈

```text
C++17 或 C++20
CMake
STL
Eigen，可选
GoogleTest
Google Benchmark
OpenCV，可选
yaml-cpp，可选
nlohmann/json，可选
```

### 4.2 Python 技术栈

```text
Python 3.10+
numpy
pandas
matplotlib
opencv-python，可选
pyyaml
imageio
```

### 4.3 可选 ROS2 技术栈

```text
ROS2 Humble / Iron / Rolling
rclcpp
nav_msgs
geometry_msgs
visualization_msgs
tf2
rviz2
```

---

## 5. 推荐项目结构

```text
AutoPlanner/
├── CMakeLists.txt
├── README.md
├── LICENSE
├── .gitignore
├── .clang-format
├── .clang-tidy
├── plan.md
│
├── cmake/
│   ├── CompilerOptions.cmake
│   ├── Sanitizers.cmake
│   └── Dependencies.cmake
│
├── docs/
│   ├── design.md
│   ├── algorithms.md
│   ├── benchmark_report.md
│   ├── api_reference.md
│   ├── roadmap.md
│   └── images/
│       ├── astar_demo.png
│       ├── improved_astar_demo.png
│       ├── rrt_demo.png
│       ├── rrt_star_demo.png
│       ├── smoothing_demo.png
│       └── benchmark_plots/
│
├── include/
│   └── autoplanner/
│       ├── autoplanner.h
│       │
│       ├── core/
│       │   ├── point.h
│       │   ├── pose.h
│       │   ├── node.h
│       │   ├── grid_map.h
│       │   ├── path.h
│       │   ├── planner_base.h
│       │   ├── planner_result.h
│       │   ├── planner_status.h
│       │   └── types.h
│       │
│       ├── planners/
│       │   ├── graph_search/
│       │   │   ├── dijkstra.h
│       │   │   ├── astar.h
│       │   │   ├── weighted_astar.h
│       │   │   ├── improved_astar.h
│       │   │   ├── jps.h
│       │   │   └── dstar_lite.h
│       │   │
│       │   ├── sampling/
│       │   │   ├── rrt.h
│       │   │   ├── rrt_star.h
│       │   │   ├── informed_rrt_star.h
│       │   │   └── bi_rrt.h
│       │   │
│       │   └── kinodynamic/
│       │       └── hybrid_astar.h
│       │
│       ├── costmap/
│       │   ├── costmap_2d.h
│       │   ├── obstacle_inflation.h
│       │   ├── distance_transform.h
│       │   └── obstacle_cost_layer.h
│       │
│       ├── collision/
│       │   ├── collision_checker.h
│       │   ├── grid_collision_checker.h
│       │   └── line_collision_checker.h
│       │
│       ├── smoothing/
│       │   ├── path_smoother.h
│       │   ├── shortcut_smoother.h
│       │   ├── bezier_smoother.h
│       │   ├── bspline_smoother.h
│       │   └── gradient_smoother.h
│       │
│       ├── heuristics/
│       │   ├── heuristic.h
│       │   ├── manhattan.h
│       │   ├── euclidean.h
│       │   ├── diagonal.h
│       │   └── reeds_shepp.h
│       │
│       ├── metrics/
│       │   ├── path_metrics.h
│       │   ├── benchmark_metrics.h
│       │   └── planner_profiler.h
│       │
│       ├── io/
│       │   ├── map_loader.h
│       │   ├── map_saver.h
│       │   ├── path_writer.h
│       │   ├── json_writer.h
│       │   └── config_loader.h
│       │
│       └── utils/
│           ├── timer.h
│           ├── logger.h
│           ├── random.h
│           ├── math_utils.h
│           └── priority_queue.h
│
├── src/
│   ├── core/
│   │   ├── grid_map.cpp
│   │   ├── path.cpp
│   │   ├── planner_result.cpp
│   │   └── types.cpp
│   │
│   ├── planners/
│   │   ├── graph_search/
│   │   │   ├── dijkstra.cpp
│   │   │   ├── astar.cpp
│   │   │   ├── weighted_astar.cpp
│   │   │   ├── improved_astar.cpp
│   │   │   ├── jps.cpp
│   │   │   └── dstar_lite.cpp
│   │   │
│   │   ├── sampling/
│   │   │   ├── rrt.cpp
│   │   │   ├── rrt_star.cpp
│   │   │   ├── informed_rrt_star.cpp
│   │   │   └── bi_rrt.cpp
│   │   │
│   │   └── kinodynamic/
│   │       └── hybrid_astar.cpp
│   │
│   ├── costmap/
│   │   ├── costmap_2d.cpp
│   │   ├── obstacle_inflation.cpp
│   │   ├── distance_transform.cpp
│   │   └── obstacle_cost_layer.cpp
│   │
│   ├── collision/
│   │   ├── grid_collision_checker.cpp
│   │   └── line_collision_checker.cpp
│   │
│   ├── smoothing/
│   │   ├── shortcut_smoother.cpp
│   │   ├── bezier_smoother.cpp
│   │   ├── bspline_smoother.cpp
│   │   └── gradient_smoother.cpp
│   │
│   ├── heuristics/
│   │   ├── manhattan.cpp
│   │   ├── euclidean.cpp
│   │   ├── diagonal.cpp
│   │   └── reeds_shepp.cpp
│   │
│   ├── metrics/
│   │   ├── path_metrics.cpp
│   │   ├── benchmark_metrics.cpp
│   │   └── planner_profiler.cpp
│   │
│   ├── io/
│   │   ├── map_loader.cpp
│   │   ├── map_saver.cpp
│   │   ├── path_writer.cpp
│   │   ├── json_writer.cpp
│   │   └── config_loader.cpp
│   │
│   └── utils/
│       ├── timer.cpp
│       ├── logger.cpp
│       ├── random.cpp
│       └── math_utils.cpp
│
├── apps/
│   ├── autoplanner_cli.cpp
│   ├── run_single_planner.cpp
│   ├── run_benchmark.cpp
│   ├── compare_planners.cpp
│   └── generate_costmap.cpp
│
├── examples/
│   ├── astar_example.cpp
│   ├── improved_astar_example.cpp
│   ├── rrt_example.cpp
│   ├── rrt_star_example.cpp
│   ├── smoothing_example.cpp
│   └── costmap_example.cpp
│
├── tests/
│   ├── CMakeLists.txt
│   ├── test_grid_map.cpp
│   ├── test_map_loader.cpp
│   ├── test_dijkstra.cpp
│   ├── test_astar.cpp
│   ├── test_improved_astar.cpp
│   ├── test_rrt.cpp
│   ├── test_rrt_star.cpp
│   ├── test_collision_checker.cpp
│   ├── test_smoothing.cpp
│   └── test_metrics.cpp
│
├── benchmark/
│   ├── CMakeLists.txt
│   ├── benchmark_astar.cpp
│   ├── benchmark_improved_astar.cpp
│   ├── benchmark_rrt.cpp
│   ├── benchmark_rrt_star.cpp
│   └── benchmark_all.cpp
│
├── data/
│   ├── maps/
│   │   ├── simple_50x50.txt
│   │   ├── maze_100x100.txt
│   │   ├── warehouse_100x100.txt
│   │   ├── random_100x100_density_10.txt
│   │   ├── random_100x100_density_20.txt
│   │   ├── random_200x200_density_20.txt
│   │   └── dynamic_obstacles/
│   │       ├── frame_000.txt
│   │       ├── frame_001.txt
│   │       └── frame_002.txt
│   │
│   ├── png_maps/
│   │   ├── simple.png
│   │   ├── maze.png
│   │   └── warehouse.png
│   │
│   └── configs/
│       ├── astar.yaml
│       ├── improved_astar.yaml
│       ├── rrt.yaml
│       ├── rrt_star.yaml
│       ├── hybrid_astar.yaml
│       └── benchmark.yaml
│
├── scripts/
│   ├── generate_random_map.py
│   ├── generate_warehouse_map.py
│   ├── visualize_path.py
│   ├── visualize_costmap.py
│   ├── plot_benchmark.py
│   ├── run_all_experiments.py
│   ├── compare_results.py
│   └── make_gif.py
│
├── results/
│   ├── paths/
│   ├── images/
│   ├── metrics/
│   ├── benchmark/
│   └── gifs/
│
├── python/
│   └── bindings/
│       ├── CMakeLists.txt
│       └── py_autoplanner.cpp
│
└── ros2_ws/
    └── src/
        └── autoplanner_ros/
            ├── CMakeLists.txt
            ├── package.xml
            ├── include/
            ├── src/
            │   ├── planner_node.cpp
            │   └── map_subscriber.cpp
            └── launch/
                └── planner_demo.launch.py
```

---

# 6. 核心 C++ 类设计

## 6.1 Point / Pose

```cpp
struct Point2i {
    int x;
    int y;
};

struct Point2d {
    double x;
    double y;
};

struct Pose2d {
    double x;
    double y;
    double theta;
};
```

---

## 6.2 GridMap

职责：

```text
1. 读取地图
2. 保存地图数据
3. 判断点是否在地图内
4. 判断点是否为障碍物
5. 提供地图宽度、高度、分辨率
```

建议接口：

```cpp
class GridMap {
public:
    bool loadFromTxt(const std::string& file_path);
    bool loadFromPng(const std::string& file_path);

    bool isInside(int x, int y) const;
    bool isFree(int x, int y) const;
    bool isOccupied(int x, int y) const;

    int width() const;
    int height() const;

    void setResolution(double resolution);
    double resolution() const;

private:
    int width_;
    int height_;
    double resolution_;
    std::vector<int> data_;
};
```

---

## 6.3 PlannerBase

所有规划器继承统一接口。

```cpp
class PlannerBase {
public:
    virtual ~PlannerBase() = default;

    virtual PlannerResult plan(
        const GridMap& map,
        const Point2i& start,
        const Point2i& goal
    ) = 0;

    virtual std::string name() const = 0;
};
```

意义：

```text
1. 统一所有算法调用方式
2. 方便 benchmark
3. 方便后续扩展新算法
4. 方便封装 ROS2 接口
```

---

## 6.4 PlannerResult

保存规划结果和指标。

```cpp
struct PlannerResult {
    bool success = false;
    std::string planner_name;

    std::vector<Point2d> path;

    double path_length = 0.0;
    double planning_time_ms = 0.0;
    int expanded_nodes = 0;
    int iterations = 0;

    int turning_count = 0;
    double smoothness_score = 0.0;
    double min_obstacle_distance = 0.0;
    double average_curvature = 0.0;

    bool collision_free = true;

    std::string message;
};
```

---

## 6.5 Costmap2D

职责：

```text
1. 根据 GridMap 生成代价地图
2. 障碍物膨胀
3. 距离变换
4. 计算靠近障碍物的风险代价
```

建议接口：

```cpp
class Costmap2D {
public:
    void buildFromGridMap(const GridMap& map);
    void inflateObstacles(double robot_radius);
    void computeDistanceTransform();

    double obstacleCost(int x, int y) const;
    double distanceToObstacle(int x, int y) const;

private:
    int width_;
    int height_;
    std::vector<double> cost_;
    std::vector<double> distance_;
};
```

---

## 6.6 CollisionChecker

职责：

```text
1. 点碰撞检测
2. 线段碰撞检测
3. 路径碰撞检测
4. 支持 RRT / RRT* / 平滑模块
```

建议接口：

```cpp
class CollisionChecker {
public:
    explicit CollisionChecker(const GridMap& map);

    bool isStateValid(const Point2d& p) const;
    bool isSegmentValid(const Point2d& p1, const Point2d& p2) const;

private:
    const GridMap& map_;
};
```

---

# 7. 算法实现规划

## 7.1 Dijkstra

定位：

```text
最基础的无启发式最短路算法，用作 baseline。
```

核心步骤：

```text
1. 初始化所有节点 g_score 为无穷大
2. 起点 g_score = 0
3. 使用 priority_queue 选择当前代价最小节点
4. 遍历邻居
5. 更新邻居代价
6. 记录 parent
7. 到达终点后回溯路径
```

优点：

```text
1. 保证最短路径
2. 实现简单
3. 适合验证工程基础是否正确
```

缺点：

```text
1. 没有启发函数
2. 扩展节点多
3. 大地图速度较慢
```

---

## 7.2 A*

定位：

```text
项目最重要 baseline。
```

公式：

```text
f(n) = g(n) + h(n)
```

含义：

```text
g(n)：从起点到当前节点的真实代价
h(n)：当前节点到目标点的启发式估计
f(n)：节点优先级
```

启发函数：

```text
1. Manhattan Distance
2. Euclidean Distance
3. Diagonal Distance
```

优点：

```text
1. 比 Dijkstra 快
2. 结果稳定
3. 适合栅格地图路径规划
```

缺点：

```text
1. 容易贴近障碍物
2. 输出路径转折多
3. 对机器人执行不够友好
```

---

## 7.3 Weighted A*

公式：

```text
f(n) = g(n) + w * h(n)
```

意义：

```text
通过增加启发函数权重，让算法更偏向目标点，减少搜索范围。
```

实验参数：

```text
w = 1.0
w = 1.2
w = 1.5
w = 2.0
```

对比指标：

```text
1. 搜索速度是否提升
2. 路径长度是否增加
3. 扩展节点数是否减少
```

---

## 7.4 Improved A*

这是本项目最重要的自定义算法设计点。

目标：

```text
解决传统 A* 路径贴近障碍物、转折多、不够平滑的问题。
```

改进公式：

```text
f(n) = g(n)
     + w_h * h(n)
     + w_obs * obstacle_cost(n)
     + w_turn * turning_cost(n)
```

参数说明：

```text
g(n)：起点到当前节点的真实代价
h(n)：当前节点到目标点的启发式距离
obstacle_cost(n)：靠近障碍物的风险代价
turning_cost(n)：转弯惩罚
w_h：启发函数权重
w_obs：障碍物风险权重
w_turn：转弯惩罚权重
```

预期效果：

```text
普通 A*：
- 路径可能较短
- 可能贴近障碍物
- 转弯较多

Improved A*：
- 路径可能略长
- 离障碍物更远
- 转弯更少
- 更适合机器人跟踪
```

消融实验：

```text
1. A*
   f = g + h

2. Weighted A*
   f = g + w_h * h

3. A* + obstacle cost
   f = g + h + w_obs * obstacle_cost

4. A* + turning cost
   f = g + h + w_turn * turning_cost

5. Improved A*
   f = g + w_h * h + w_obs * obstacle_cost + w_turn * turning_cost
```

评估指标：

```text
path_length
planning_time_ms
expanded_nodes
turning_count
minimum_obstacle_distance
average_curvature
smoothness_score
```

---

## 7.5 JPS

全称：

```text
Jump Point Search
```

定位：

```text
在规则栅格地图上加速 A*。
```

核心概念：

```text
1. 跳点
2. 强制邻居
3. 递归跳跃
4. 减少无意义节点扩展
```

实验目标：

```text
比较 A* 和 JPS 在大地图上的 expanded_nodes 和 planning_time_ms。
```

---

## 7.6 D* Lite

定位：

```text
动态环境中的增量重规划算法。
```

适用场景：

```text
1. 障碍物变化
2. 地图局部更新
3. 移动机器人在线导航
```

优先级：

```text
可选后期实现，不作为第一阶段重点。
```

---

## 7.7 RRT

全称：

```text
Rapidly-exploring Random Tree
```

定位：

```text
连续空间采样规划 baseline。
```

核心流程：

```text
1. 初始化树，加入起点
2. 随机采样 q_rand
3. 找最近节点 q_near
4. 沿 q_near 到 q_rand 方向扩展 step_size，得到 q_new
5. 检查 q_near 到 q_new 是否碰撞
6. 若无碰撞，则加入树
7. 若 q_new 接近 goal，则尝试连接 goal
8. 回溯路径
```

关键参数：

```text
max_iterations
step_size
goal_sample_rate
goal_tolerance
collision_check_resolution
```

优点：

```text
1. 适合连续空间
2. 实现相对简单
3. 能处理复杂障碍环境
```

缺点：

```text
1. 路径质量不稳定
2. 路径不一定最优
3. 随机性较强
```

---

## 7.8 RRT*

定位：

```text
RRT 的渐近最优版本。
```

相较 RRT 增加：

```text
1. near nodes 搜索
2. choose parent
3. rewire
4. cost 更新
```

实验重点：

```text
1. RRT* 是否比 RRT 路径更短
2. RRT* 路径长度是否随迭代次数下降
3. RRT* 的规划耗时是否显著增加
```

---

## 7.9 Informed RRT*

定位：

```text
RRT* 的高阶优化版本。
```

核心思想：

```text
找到第一条可行路径后，将采样空间限制在连接起点和终点的椭圆区域内，提高优化效率。
```

优先级：

```text
进阶功能，适合在 RRT* 完成后实现。
```

---

## 7.10 Bi-RRT

定位：

```text
从起点和终点同时生长两棵树，提高搜索速度。
```

优先级：

```text
可选。
```

---

## 7.11 Hybrid A*

定位：

```text
考虑车辆运动学约束的路径规划算法。
```

普通 A* 状态：

```text
(x, y)
```

Hybrid A* 状态：

```text
(x, y, theta)
```

动作集合：

```text
forward-left
forward-straight
forward-right
backward-left
backward-straight
backward-right
```

意义：

```text
更接近自动驾驶、AGV、移动机器人运动规划。
```

优先级：

```text
冲刺阶段实现。
```

---

# 8. 路径平滑规划

## 8.1 为什么需要路径平滑

图搜索算法输出的路径通常是折线，存在以下问题：

```text
1. 转折点多
2. 曲率不连续
3. 机器人难以直接跟踪
4. 控制输入可能抖动
```

因此需要路径平滑模块。

---

## 8.2 Shortcut Smoothing

思路：

```text
随机选择路径中的两个点，如果这两个点之间的直线无碰撞，则删除中间路径点。
```

优点：

```text
1. 简单
2. 快速
3. 效果明显
```

优先级：

```text
必须实现。
```

---

## 8.3 Bezier Smoothing

思路：

```text
使用 Bezier 曲线拟合路径，使路径更平滑。
```

优点：

```text
1. 可视化效果好
2. 适合展示
3. 适合写入 README
```

---

## 8.4 B-spline Smoothing

思路：

```text
使用 B-spline 生成局部可控的平滑曲线。
```

优点：

```text
1. 平滑性更好
2. 局部修改不会影响全局太多
3. 更接近轨迹规划
```

---

## 8.5 Gradient-based Smoothing

目标函数：

```text
cost = path_smooth_cost + obstacle_cost + curvature_cost
```

意义：

```text
更接近优化式轨迹规划，可作为后期扩展。
```

---

# 9. Costmap 与障碍物膨胀

## 9.1 为什么需要障碍物膨胀

机器人不是一个点，而是有半径和尺寸的实体。

如果只用原始地图规划，路径可能贴着障碍物边缘走，真实机器人执行时可能发生碰撞。

因此需要：

```text
1. obstacle inflation
2. distance transform
3. obstacle cost layer
```

---

## 9.2 障碍物膨胀

输入：

```text
robot_radius
map_resolution
```

膨胀半径：

```text
inflation_cells = ceil(robot_radius / resolution)
```

效果：

```text
原始障碍物周围若干格也被视为不可通行或高代价区域。
```

---

## 9.3 距离变换

目标：

```text
计算每个 free cell 到最近障碍物的距离。
```

用途：

```text
1. 计算 obstacle_cost
2. 评估 minimum_obstacle_distance
3. 让 Improved A* 更倾向远离障碍物
```

---

## 9.4 障碍物风险代价

可设计为：

```text
obstacle_cost = 1 / (distance_to_obstacle + epsilon)
```

或者：

```text
obstacle_cost = exp(-distance_to_obstacle / sigma)
```

---

# 10. 地图格式设计

## 10.1 txt 地图

示例：

```text
000000000000000
011111000001110
000001000000010
000001111110010
000000000000010
011111111000010
000000000000000
```

含义：

```text
0 = free
1 = obstacle
```

---

## 10.2 png 地图，可选

含义：

```text
白色 = free
黑色 = obstacle
```

优先级：

```text
后期实现。
```

---

# 11. 配置文件设计

## 11.1 astar.yaml

```yaml
planner: astar

map:
  resolution: 0.05

robot:
  radius: 0.25

astar:
  connectivity: 8
  heuristic: euclidean
  heuristic_weight: 1.0
  allow_diagonal: true

output:
  save_path: true
  save_metrics: true
  save_visualization: true
```

---

## 11.2 improved_astar.yaml

```yaml
planner: improved_astar

map:
  resolution: 0.05

robot:
  radius: 0.25
  inflation_radius: 0.35

astar:
  connectivity: 8
  heuristic: euclidean
  heuristic_weight: 1.0
  obstacle_weight: 2.0
  turning_weight: 0.5
  allow_diagonal: true

output:
  save_path: true
  save_metrics: true
  save_visualization: true
```

---

## 11.3 rrt.yaml

```yaml
planner: rrt

rrt:
  max_iterations: 5000
  step_size: 2.0
  goal_sample_rate: 0.1
  goal_tolerance: 2.0
  collision_check_resolution: 0.5

robot:
  radius: 0.25

output:
  save_tree: true
  save_path: true
  save_metrics: true
```

---

## 11.4 rrt_star.yaml

```yaml
planner: rrt_star

rrt_star:
  max_iterations: 8000
  step_size: 2.0
  goal_sample_rate: 0.1
  goal_tolerance: 2.0
  rewire_radius: 5.0
  collision_check_resolution: 0.5

robot:
  radius: 0.25

output:
  save_tree: true
  save_path: true
  save_metrics: true
```

---

# 12. 命令行程序设计

## 12.1 autoplanner_cli

用途：

```text
通用命令行入口。
```

示例：

```bash
./build/apps/autoplanner_cli \
  --planner astar \
  --map data/maps/maze_100x100.txt \
  --start 5 5 \
  --goal 90 90 \
  --output results/astar_maze
```

---

## 12.2 run_single_planner

用途：

```text
调试单个算法。
```

示例：

```bash
./build/apps/run_single_planner \
  --planner improved_astar \
  --map data/maps/warehouse_100x100.txt \
  --start 3 5 \
  --goal 90 80 \
  --config data/configs/improved_astar.yaml \
  --output results/improved_astar_warehouse
```

---

## 12.3 run_benchmark

用途：

```text
批量运行 benchmark。
```

示例：

```bash
./build/apps/run_benchmark \
  --config data/configs/benchmark.yaml \
  --output results/benchmark/all_results.csv
```

---

# 13. Python 脚本规划

## 13.1 generate_random_map.py

功能：

```text
生成随机障碍物地图。
```

参数：

```text
width
height
obstacle_density
seed
output_path
```

---

## 13.2 generate_warehouse_map.py

功能：

```text
生成仓储地图。
```

特点：

```text
1. 货架区域
2. 通道区域
3. 起点终点可随机生成
```

---

## 13.3 visualize_path.py

功能：

```text
读取地图和路径，生成路径可视化图。
```

示例：

```bash
python scripts/visualize_path.py \
  --map data/maps/warehouse_100x100.txt \
  --path results/paths/improved_astar_path.json \
  --output results/images/improved_astar_warehouse.png
```

---

## 13.4 plot_benchmark.py

功能：

```text
读取 benchmark csv，绘制对比图。
```

输出：

```text
planning_time_compare.png
path_length_compare.png
expanded_nodes_compare.png
success_rate_compare.png
```

---

## 13.5 run_all_experiments.py

功能：

```text
批量运行所有地图和所有算法。
```

输出：

```text
results/benchmark/all_results.csv
results/benchmark/summary.json
```

---

## 13.6 make_gif.py

功能：

```text
根据搜索过程或 RRT 采样树生成 gif 动图。
```

用于 README 展示。

---

# 14. Benchmark 设计

## 14.1 地图集合

```text
simple_50x50
maze_100x100
warehouse_100x100
random_100x100_density_10
random_100x100_density_20
random_100x100_density_30
random_200x200_density_20
random_500x500_density_20
```

---

## 14.2 算法集合

基础版本：

```text
Dijkstra
A*
Weighted A*
Improved A*
```

进阶版本：

```text
JPS
RRT
RRT*
Informed RRT*
Hybrid A*
```

---

## 14.3 指标集合

```text
planner_name
map_name
success
planning_time_ms
path_length
expanded_nodes
iterations
turning_count
smoothness_score
minimum_obstacle_distance
average_curvature
collision_free
memory_usage_mb
```

---

## 14.4 图表输出

```text
1. 不同算法规划时间对比图
2. 不同算法路径长度对比图
3. 不同算法扩展节点数对比图
4. 不同算法成功率对比图
5. 不同障碍物密度下成功率对比图
6. Improved A* 消融实验图
7. 路径平滑前后对比图
8. RRT* 迭代次数与路径长度关系图
```

---

# 15. 实验结论预期

后续报告可以围绕以下结论写：

```text
1. Dijkstra 不依赖启发函数，但扩展节点多，效率低。
2. A* 利用启发函数减少搜索空间，速度明显快于 Dijkstra。
3. Weighted A* 可以进一步加快搜索，但路径可能变长。
4. Improved A* 通过障碍物风险代价提高路径安全距离。
5. 转弯惩罚可以减少路径折线程度。
6. Shortcut smoothing 可以减少冗余拐点。
7. RRT 在复杂连续空间中更灵活，但路径质量不稳定。
8. RRT* 通过 rewiring 可以逐步优化路径，但计算时间更长。
9. JPS 在规则栅格地图中可以显著减少扩展节点。
10. Hybrid A* 能生成更符合车辆运动学约束的路径。
```

---

# 16. 单元测试规划

必须测试：

```text
1. GridMap 是否能正确读取地图
2. GridMap 是否能正确判断地图边界
3. GridMap 是否能正确判断障碍物
4. Dijkstra 是否能在简单地图找到路径
5. A* 是否能在简单地图找到路径
6. A* 与 Dijkstra 在相同代价下路径长度是否一致
7. Weighted A* 是否能找到可行路径
8. Improved A* 是否能输出无碰撞路径
9. CollisionChecker 是否能检测点碰撞
10. CollisionChecker 是否能检测线段碰撞
11. ShortcutSmoother 是否能减少路径点
12. PathMetrics 是否能正确计算路径长度
13. RRT 是否能在简单地图中找到路径
14. RRT* 是否能随迭代优化路径
```

---

# 17. CMake 构建规划

## 17.1 构建命令

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

---

## 17.2 运行测试

```bash
ctest --test-dir build --output-on-failure
```

---

## 17.3 Debug 构建

```bash
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug -j
```

---

# 18. 开发路线图

## 18.1 v0.1：能跑

```text
1. CMake 工程
2. GridMap
3. Point / Pose / Path
4. Dijkstra
5. A*
6. path 输出
7. 基础 metrics
8. 简单地图
```

验收标准：

```text
能在 simple_50x50.txt 上从起点规划到终点，并输出 path 和 metrics。
```

---

## 18.2 v0.2：像机器人项目

```text
1. 8 邻域
2. 多启发函数
3. 障碍物膨胀
4. Costmap2D
5. CollisionChecker
6. PathMetrics
```

验收标准：

```text
能体现机器人半径，规划出的路径不会贴着障碍物。
```

---

## 18.3 v0.3：有算法改进

```text
1. Weighted A*
2. Improved A*
3. obstacle cost
4. turning cost
5. 消融实验
```

验收标准：

```text
能证明 Improved A* 相比 A* 更安全或更平滑。
```

---

## 18.4 v0.4：加入采样规划

```text
1. RRT
2. RRT*
3. goal bias
4. rewiring
5. RRT 树可视化
```

验收标准：

```text
能在复杂地图中输出 RRT / RRT* 路径，并比较路径质量。
```

---

## 18.5 v0.5：完整项目展示

```text
1. benchmark
2. Python 可视化
3. README
4. docs
5. 实验图
6. 简历描述
```

验收标准：

```text
项目可以作为 GitHub 展示项目。
```

---

## 18.6 v1.0：进阶扩展

```text
1. JPS
2. Informed RRT*
3. Hybrid A*
4. 动态障碍重规划
5. ROS2 wrapper
```

验收标准：

```text
项目具备更完整的机器人算法工程特征。
```

---

# 19. 8 周开发计划

## Week 1：工程骨架

```text
1. 创建项目目录
2. 编写 CMakeLists.txt
3. 编写 Point / Pose / Path
4. 编写 GridMap
5. 编写 MapLoader
6. 准备 simple_50x50.txt
7. 编写基础 README
```

---

## Week 2：Dijkstra + A*

```text
1. 实现 PlannerBase
2. 实现 PlannerResult
3. 实现 Dijkstra
4. 实现 A*
5. 实现路径回溯
6. 实现规划时间统计
7. 添加 test_astar
```

---

## Week 3：启发函数与指标

```text
1. Manhattan heuristic
2. Euclidean heuristic
3. Diagonal heuristic
4. 4 邻域和 8 邻域
5. path_length
6. expanded_nodes
7. turning_count
8. metrics.json
```

---

## Week 4：Costmap + Improved A*

```text
1. 障碍物膨胀
2. distance transform
3. obstacle cost
4. turning cost
5. Improved A*
6. A* 消融实验
```

---

## Week 5：Python 可视化 + benchmark

```text
1. visualize_path.py
2. generate_random_map.py
3. run_benchmark.cpp
4. plot_benchmark.py
5. all_results.csv
6. planning_time 对比图
7. path_length 对比图
```

---

## Week 6：RRT / RRT*

```text
1. CollisionChecker
2. RRT
3. RRT*
4. goal bias
5. rewiring
6. RRT 树可视化
```

---

## Week 7：路径平滑 + JPS

```text
1. Shortcut smoothing
2. Bezier smoothing
3. B-spline smoothing
4. JPS
5. smoothing 对比实验
```

---

## Week 8：项目包装

```text
1. 完善 README
2. 完善 docs/design.md
3. 完善 docs/algorithms.md
4. 完善 docs/benchmark_report.md
5. 整理 results/images
6. 准备简历描述
7. 可选开始 ROS2 wrapper
```

---

# 20. 当前最优先 TODO

```text
[ ] 写顶层 CMakeLists.txt
[ ] 写 include/autoplanner/core/point.h
[ ] 写 include/autoplanner/core/pose.h
[ ] 写 include/autoplanner/core/grid_map.h
[ ] 写 src/core/grid_map.cpp
[ ] 写 include/autoplanner/core/path.h
[ ] 写 include/autoplanner/core/planner_result.h
[ ] 写 include/autoplanner/core/planner_base.h
[ ] 写 Dijkstra
[ ] 写 A*
[ ] 写 apps/run_single_planner.cpp
[ ] 写 data/maps/simple_50x50.txt
[ ] 写 scripts/visualize_path.py
[ ] 写 README.md 初版
```

---

# 21. README 展示规划

README 应包含：

```text
1. 项目简介
2. 项目亮点
3. 支持算法
4. 项目结构
5. 环境依赖
6. 编译方法
7. 快速开始
8. 运行示例
9. 可视化结果
10. benchmark 结果
11. 算法说明
12. 后续计划
13. 简历写法
```

README 中建议展示：

```text
astar_demo.png
improved_astar_demo.png
rrt_demo.png
rrt_star_demo.png
smoothing_before_after.png
planning_time_compare.png
path_length_compare.png
expanded_nodes_compare.png
success_rate_compare.png
```

---

# 22. 简历写法

## 22.1 核心版写法

```text
AutoPlanner：面向移动机器人的 C++ 路径规划算法库

基于 C++17 / CMake 设计并实现二维移动机器人路径规划算法库，支持 Dijkstra、A*、Weighted A* 与 Improved A* 等图搜索算法；构建统一 Planner 接口、栅格地图读取、路径输出、规划指标统计和单元测试模块；使用 Python 完成地图生成、路径可视化和多算法性能对比。
```

---

## 22.2 增强版写法

```text
针对传统 A* 易贴近障碍物、路径转折多的问题，引入障碍物风险代价与转弯惩罚项，设计 Improved A* 规划器；结合障碍物膨胀、距离变换和路径平滑模块，提高路径安全性和平滑性；通过规划时间、路径长度、扩展节点数、最小障碍物距离和转弯次数等指标完成消融实验。
```

---

## 22.3 进阶版写法

```text
进一步实现 RRT、RRT* 与 Informed RRT* 等采样规划算法，支持连续空间路径搜索、线段碰撞检测、goal bias 与 rewiring 优化；构建 benchmark 系统，对不同地图规模和障碍物密度下的算法成功率、规划耗时、路径长度和节点数量进行系统评估。
```

---

## 22.4 ROS2 版写法

```text
将 AutoPlanner 封装为 ROS2 路径规划节点，支持地图订阅、目标点输入、路径发布和 RViz 可视化，实现从算法库到机器人导航模块的工程化扩展。
```

---

# 23. 与后续项目的衔接

## 23.1 与 AutoMPC 衔接

AutoPlanner 输出全局路径，AutoMPC 可以负责轨迹跟踪。

后续项目：

```text
AutoMPC：基于 C++ 的移动机器人模型预测控制库
```

内容：

```text
1. PID
2. Pure Pursuit
3. Stanley
4. LQR
5. MPC
6. OSQP
7. 轨迹跟踪误差评估
```

---

## 23.2 与 TD-MPC2 / MBRL 衔接

AutoPlanner 的工程结构可以复用于 C++ 模型预测强化学习规划器。

后续项目：

```text
Cpp-MBRL-Planner：面向 TD-MPC2 的 C++ 模型预测强化学习规划引擎
```

流程：

```text
current_state
    ↓
C++ planner samples action sequences
    ↓
ONNX model predicts latent dynamics
    ↓
evaluate reward + value
    ↓
CEM / MPPI selects best action
    ↓
execute action
```

可实现模块：

```text
1. CEM
2. MPPI
3. shooting-based MPC
4. ONNX Runtime
5. 多线程候选轨迹采样
6. uncertainty penalty
7. safety penalty
8. warm-start planning
```

---

# 24. 项目最终完成标准

最低完成标准：

```text
1. Dijkstra
2. A*
3. Improved A*
4. GridMap
5. Costmap
6. CollisionChecker
7. PathMetrics
8. Python 可视化
9. benchmark
10. README
```

较完整完成标准：

```text
1. Dijkstra
2. A*
3. Weighted A*
4. Improved A*
5. JPS
6. RRT
7. RRT*
8. Shortcut smoothing
9. Bezier smoothing
10. benchmark_report.md
```

优秀完成标准：

```text
1. Informed RRT*
2. Hybrid A*
3. 动态障碍重规划
4. ROS2 wrapper
5. GIF 动图
6. 完整实验报告
7. GitHub 展示图
8. 可直接写入简历
```

---

# 25. 最终建议

当前阶段不要一开始就追求 ROS2、TD-MPC2 或复杂仿真环境。

最稳妥路线：

```text
A* 
→ Dijkstra 
→ Weighted A* 
→ Improved A* 
→ Costmap 
→ Path Smoothing 
→ Benchmark 
→ RRT 
→ RRT* 
→ JPS 
→ Hybrid A* 
→ ROS2
```

这个路线从简单到复杂，既能保证项目持续产出，也能逐步增加算法深度和工程含金量。

最终 AutoPlanner 可以作为你的第一个核心 C++ 算法工程项目，为后续 AutoMPC 和 TD-MPC2 C++ Planning Engine 打基础。



