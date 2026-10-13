#include "rrt_planner.h"
#include <cmath>
#include <limits>
#include <algorithm>

namespace amass {
namespace global_planner {

RRTPlanner::RRTPlanner()
    : max_iter_(2000), step_size_(1.0), goal_sample_rate_(0.05), goal_tolerance_(1.5)
{
    // 可根据地图实际范围调整
    map_min_x_ = 0.0; map_min_y_ = 0.0; map_max_x_ = 20.0; map_max_y_ = 20.0;
}

void RRTPlanner::toGrid(double wx, double wy, const costmap_2d::Costmap2D* costmap, int& mx, int& my) const {
    mx = static_cast<int>((wx - costmap->getOriginX()) / costmap->getResolution());
    my = static_cast<int>((wy - costmap->getOriginY()) / costmap->getResolution());
}

double RRTPlanner::distance(const RRTNode& n1, const RRTNode& n2) const {
    return std::hypot(n1.x - n2.x, n1.y - n2.y);
}

bool RRTPlanner::isCollisionFree(const RRTNode& n1, const RRTNode& n2, const costmap_2d::Costmap2D* costmap) const {
    // Bresenham直线检查
    int mx1, my1, mx2, my2;
    toGrid(n1.x, n1.y, costmap, mx1, my1);
    toGrid(n2.x, n2.y, costmap, mx2, my2);

    int dx = std::abs(mx2 - mx1), dy = std::abs(my2 - my1);
    int sx = mx1 < mx2 ? 1 : -1;
    int sy = my1 < my2 ? 1 : -1;
    int err = dx - dy;

    int x = mx1, y = my1;
    while (true) {
        if (!isFree(costmap, x, y)) return false;
        if (x == mx2 && y == my2) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 < dx)  { err += dx; y += sy; }
    }
    return true;
}

RRTNode RRTPlanner::sampleRandomNode(const costmap_2d::Costmap2D* costmap, const geometry_msgs::PoseStamped& goal) {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    if (dist(rng_) < goal_sample_rate_) {
        return RRTNode(goal.pose.position.x, goal.pose.position.y, -1, -1, 0);
    }
    std::uniform_real_distribution<double> dist_x(map_min_x_, map_max_x_);
    std::uniform_real_distribution<double> dist_y(map_min_y_, map_max_y_);
    return RRTNode(dist_x(rng_), dist_y(rng_), -1, -1, 0);
}

int RRTPlanner::getNearestNodeId(const std::unordered_map<int, RRTNode>& nodes, const RRTNode& node, int map_width) const {
    double min_dist = std::numeric_limits<double>::max();
    int nearest_id = -1;
    for (const auto& kv : nodes) {
        double d = distance(kv.second, node);
        if (d < min_dist) {
            min_dist = d;
            nearest_id = kv.first;
        }
    }
    return nearest_id;
}

void RRTPlanner::backtracePath(const std::unordered_map<int, RRTNode>& nodes, int goal_id,
                               std::vector<geometry_msgs::PoseStamped>& plan,
                               const costmap_2d::Costmap2D* costmap,
                               const geometry_msgs::PoseStamped& goal) const {
    plan.clear();
    int id = goal_id;
    geometry_msgs::PoseStamped pose = goal;
    while (nodes.count(id)) {
        const RRTNode& n = nodes.at(id);
        pose.pose.position.x = n.x;
        pose.pose.position.y = n.y;
        plan.push_back(pose);
        if (n.parent_id == -1) break;
        id = n.parent_id;
    }
    std::reverse(plan.begin(), plan.end());
}

bool RRTPlanner::makePlan(const costmap_2d::Costmap2D* costmap,
                          const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan) {
    plan.clear();

    int map_width = costmap->getSizeInCellsX();
    int map_height = costmap->getSizeInCellsY();

    int sx, sy, gx, gy;
    if (!worldToMap(costmap, start.pose.position.x, start.pose.position.y, sx, sy) ||
        !worldToMap(costmap, goal.pose.position.x, goal.pose.position.y, gx, gy)) {
        ROS_ERROR("RRT: The starting or ending point is outside the map boundaries");
        return false;
    }

    // 起点
    RRTNode start_node(start.pose.position.x, start.pose.position.y, -1, toId(sx, sy, map_width), 0);
    std::unordered_map<int, RRTNode> nodes;
    nodes[start_node.id] = start_node;

    bool found = false;
    int goal_id = -1;

    for (int iter = 0; iter < max_iter_; ++iter) {
        RRTNode rand_node = sampleRandomNode(costmap, goal);

        // 找最近节点
        int nearest_id = getNearestNodeId(nodes, rand_node, map_width);
        if (nearest_id == -1) continue;
        RRTNode nearest = nodes[nearest_id];

        // 生成新节点
        double theta = std::atan2(rand_node.y - nearest.y, rand_node.x - nearest.x);
        double new_x = nearest.x + step_size_ * std::cos(theta);
        double new_y = nearest.y + step_size_ * std::sin(theta);

        // 边界检查
        if (new_x < map_min_x_ || new_x > map_max_x_ || new_y < map_min_y_ || new_y > map_max_y_)
            continue;

        int mx, my;
        toGrid(new_x, new_y, costmap, mx, my);
        if (mx < 0 || mx >= map_width || my < 0 || my >= map_height)
            continue;

        int new_id = toId(mx, my, map_width);
        if (nodes.count(new_id)) continue;

        RRTNode new_node(new_x, new_y, nearest.id, new_id, nearest.cost + step_size_);

        // 碰撞检测
        if (!isCollisionFree(nearest, new_node, costmap)) continue;

        nodes[new_id] = new_node;

        // 是否到达目标
        if (distance(new_node, RRTNode(goal.pose.position.x, goal.pose.position.y, -1, -1, 0)) < goal_tolerance_) {
            found = true;
            goal_id = new_id;
            break;
        }
    }

    if (!found) {
        ROS_WARN("RRT: couldn't found the path");
        return false;
    }

    backtracePath(nodes, goal_id, plan, costmap, goal);
    return true;
}

} // namespace global_planner
} // namespace amass