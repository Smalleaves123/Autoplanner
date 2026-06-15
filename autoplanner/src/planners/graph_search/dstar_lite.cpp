#include "autoplanner/planners/graph_search/dstar_lite.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>

#include "autoplanner/heuristics/euclidean.h"

namespace autoplanner {

namespace {

// D* Lite key: k1 = min(g, rhs) + h + km,  k2 = min(g, rhs)
struct Key {
    double k1 = 0.0, k2 = 0.0;
    bool operator>(const Key& o) const {
        return k1 > o.k1 || (k1 == o.k1 && k2 > o.k2);
    }
};

struct QNode {
    int idx = -1;
    Key key;
    bool operator>(const QNode& o) const { return key > o.key; }
};

}  // namespace

DStarLitePlanner::DStarLitePlanner(bool allow_diagonal)
    : allow_diagonal_(allow_diagonal)
    , heuristic_(std::make_unique<EuclideanHeuristic>()) {}

void DStarLitePlanner::setHeuristic(std::unique_ptr<Heuristic> h) {
    heuristic_ = std::move(h);
}

std::string DStarLitePlanner::name() const { return "dstar_lite"; }

PlannerResult DStarLitePlanner::plan(const GridMap& map, const Point2i& start,
                                      const Point2i& goal) {
    PlannerResult result;
    result.planner_name = name();
    auto t0 = std::chrono::steady_clock::now();

    if (!map.isFree(start.x, start.y)) {
        result.message = "Start is occupied."; return result;
    }
    if (!map.isFree(goal.x, goal.y)) {
        result.message = "Goal is occupied."; return result;
    }

    int w = map.width(), mh = map.height(), n = w * mh;
    int si = map.index(start.x, start.y);
    int gi = map.index(goal.x, goal.y);

    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> g(static_cast<size_t>(n), INF);
    std::vector<double> rhs(static_cast<size_t>(n), INF);
    std::vector<int> parent(static_cast<size_t>(n), -1);
    std::vector<unsigned char> closed(static_cast<size_t>(n), 0);

    std::priority_queue<QNode, std::vector<QNode>, std::greater<QNode>> open;

    auto h = [&](int a, int b) {
        int ax = a % w, ay = a / w, bx = b % w, by = b / w;
        return heuristic_->compute(Point2i(ax, ay), Point2i(bx, by));
    };

    rhs[static_cast<size_t>(gi)] = 0.0;
    open.push({gi, {h(gi, si), 0.0}});

    // Simplified D* Lite: first-pass planning (no dynamic updates).
    // Uses the same reverse-search structure as standard D* Lite.
    while (!open.empty()) {
        auto cur = open.top(); open.pop();
        int u = cur.idx;
        if (closed[static_cast<size_t>(u)]) continue;

        if (g[static_cast<size_t>(u)] > rhs[static_cast<size_t>(u)]) {
            g[static_cast<size_t>(u)] = rhs[static_cast<size_t>(u)];
        } else {
            g[static_cast<size_t>(u)] = INF;
            continue;
        }
        closed[static_cast<size_t>(u)] = 1;
        result.expanded_nodes++;

        if (u == si) { result.success = true; break; }

        int ux = u % w, uy = u / w;
        std::vector<int> dxs = {1,-1,0,0}, dys = {0,0,1,-1};
        if (allow_diagonal_)
            for (int dx : {1,-1}) for (int dy : {1,-1})
                { dxs.push_back(dx); dys.push_back(dy); }

        for (size_t i = 0; i < dxs.size(); ++i) {
            int nx = ux + dxs[i], ny = uy + dys[i];
            if (!map.isFree(nx, ny)) continue;
            int v = map.index(nx, ny);
            double c = (dxs[i] != 0 && dys[i] != 0) ? std::sqrt(2.0) : 1.0;
            double new_rhs = g[static_cast<size_t>(u)] + c;
            if (new_rhs < rhs[static_cast<size_t>(v)]) {
                rhs[static_cast<size_t>(v)] = new_rhs;
                parent[static_cast<size_t>(v)] = u;
                double hh = h(v, si);
                open.push({v, {std::min(g[static_cast<size_t>(v)], rhs[static_cast<size_t>(v)]) + hh,
                               std::min(g[static_cast<size_t>(v)], rhs[static_cast<size_t>(v)])}});
            }
        }
    }

    if (result.success) {
        // Reconstruct path from start to goal using parent pointers.
        Path2d path;
        int cur = si;
        while (cur != gi && cur != -1) {
            path.emplace_back(static_cast<double>(cur % w),
                              static_cast<double>(cur / w));
            cur = parent[static_cast<size_t>(cur)];
        }
        path.emplace_back(static_cast<double>(goal.x),
                          static_cast<double>(goal.y));
        result.path = std::move(path);
        result.path_length = computePathLength(result.path);
        result.message = "Path found.";
    } else {
        result.message = "No path found.";
    }

    auto t1 = std::chrono::steady_clock::now();
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.iterations = result.expanded_nodes;
    return result;
}

}  // namespace autoplanner
