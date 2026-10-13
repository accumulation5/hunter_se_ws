#include "improved_rrt_planner.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace amass {
namespace global_planner {

ImprovedRRTPlanner::ImprovedRRTPlanner()
    : step_size_(0.5), goal_tolerance_(0.3), goal_bias_prob_(0.15),
      max_iterations_(10000), corridor_width_(0.8), alpha_(0.7) {}

void ImprovedRRTPlanner::setCorridorPath(const std::vector<geometry_msgs::PoseStamped>& path) {
    corridor_path_ = path;
}

bool ImprovedRRTPlanner::makePlan(const costmap_2d::Costmap2D* costmap,
                                  const geometry_msgs::PoseStamped& start,
                                  const geometry_msgs::PoseStamped& goal,
                                  std::vector<geometry_msgs::PoseStamped>& plan) {
    plan.clear();
    tree_.clear();

    ImprovedRRTNode start_node(start.pose.position.x, start.pose.position.y, -1, 0.0);
    tree_.push_back(start_node);

    std::uniform_real_distribution<> dis(0.0, 1.0);
    std::uniform_real_distribution<> dis_x(costmap->getOriginX(), costmap->getOriginX() + costmap->getSizeInMetersX());
    std::uniform_real_distribution<> dis_y(costmap->getOriginY(), costmap->getOriginY() + costmap->getSizeInMetersY());

    int goal_id = -1;

    for (int i = 0; i < max_iterations_; ++i) {
        double rand_x, rand_y;
        // 目标偏向采样
        if (dis(rng_) < goal_bias_prob_) {
            rand_x = goal.pose.position.x;
            rand_y = goal.pose.position.y;
        } else {
            // 采样空间约束+走廊约束
            int try_count = 0;
            do {
                rand_x = dis_x(rng_);
                rand_y = dis_y(rng_);
                ++try_count;
                if (try_count > 1000) break; // 防止死循环
            } while (!isValidSample(costmap, rand_x, rand_y) || !inCorridor(rand_x, rand_y));
        }

        // 查找最近节点
        int near_id = nearestNode(rand_x, rand_y);
        ImprovedRRTNode near_node = tree_[near_id];

        // 行道中心线引导
        double dx = rand_x - near_node.x;
        double dy = rand_y - near_node.y;
        double dist = std::hypot(dx, dy);

        geometry_msgs::PoseStamped proj = getProjectionOnCenterLine(near_node.x, near_node.y);
        double guide_dx = proj.pose.position.x - near_node.x;
        double guide_dy = proj.pose.position.y - near_node.y;
        double guide_dist = std::hypot(guide_dx, guide_dy);

        // 加权方向
        double dir_x = alpha_ * (dx / (dist + 1e-6)) + (1 - alpha_) * (guide_dx / (guide_dist + 1e-6));
        double dir_y = alpha_ * (dy / (dist + 1e-6)) + (1 - alpha_) * (guide_dy / (guide_dist + 1e-6));
        double dir_norm = std::hypot(dir_x, dir_y);

        // 生成新节点
        ImprovedRRTNode new_node;
        if (dist < step_size_) {
            new_node.x = rand_x;
            new_node.y = rand_y;
        } else {
            new_node.x = near_node.x + step_size_ * (dir_x / (dir_norm + 1e-6));
            new_node.y = near_node.y + step_size_ * (dir_y / (dir_norm + 1e-6));
        }
        new_node.parent_id = near_id;
        new_node.cost = near_node.cost + step_size_;

        // 碰撞检测
        if (collisionCheck(costmap, near_node, new_node)) {
            tree_.push_back(new_node);

            // 检查是否到达目标
            double goal_dist = std::hypot(new_node.x - goal.pose.position.x, new_node.y - goal.pose.position.y);
            if (goal_dist < goal_tolerance_) {
                goal_id = tree_.size() - 1;
                break;
            }
        }
    }

    if (goal_id == -1) {
        ROS_WARN("Improved RRT failed to find path within %d iterations", max_iterations_);
        return false;
    }

    // 回溯路径
    std::vector<geometry_msgs::PoseStamped> raw_plan;
    int curr_id = goal_id;
    while (curr_id != -1) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = tree_[curr_id].x;
        pose.pose.position.y = tree_[curr_id].y;
        pose.pose.orientation.w = 1.0;
        raw_plan.push_back(pose);
        curr_id = tree_[curr_id].parent_id;
    }
    std::reverse(raw_plan.begin(), raw_plan.end());

    // 路径平滑
    smoothPath(raw_plan);
    plan = raw_plan;
    return true;
}

bool ImprovedRRTPlanner::isValidSample(const costmap_2d::Costmap2D* costmap, double x, double y) const {
    int mx, my;
    if (!AbstractAlgorithm::worldToMap(costmap, x, y, mx, my)) return false;
    unsigned char cost = costmap->getCost(mx, my);
    return cost < costmap_2d::LETHAL_OBSTACLE;
}

bool ImprovedRRTPlanner::inCorridor(double x, double y) const {
    if (corridor_path_.empty()) return true;
    double min_dist = std::numeric_limits<double>::max();
    for (const auto& p : corridor_path_) {
        double dx = x - p.pose.position.x;
        double dy = y - p.pose.position.y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) min_dist = dist;
    }
    return min_dist < corridor_width_ / 2.0;
}

int ImprovedRRTPlanner::nearestNode(double x, double y) const {
    double min_dist = std::numeric_limits<double>::max();
    int min_id = -1;
    for (size_t i = 0; i < tree_.size(); ++i) {
        double dx = x - tree_[i].x;
        double dy = y - tree_[i].y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            min_id = i;
        }
    }
    return min_id;
}

bool ImprovedRRTPlanner::collisionCheck(const costmap_2d::Costmap2D* costmap, const ImprovedRRTNode& from, const ImprovedRRTNode& to) const {
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double dist = std::hypot(dx, dy);
    int steps = static_cast<int>(dist / costmap->getResolution()) + 1;
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        double x = from.x + t * dx;
        double y = from.y + t * dy;
        if (!isValidSample(costmap, x, y)) return false;
    }
    return true;
}

// 投影到中心线（最近点）
geometry_msgs::PoseStamped ImprovedRRTPlanner::getProjectionOnCenterLine(double x, double y) const {
    geometry_msgs::PoseStamped proj;
    if (corridor_path_.empty()) {
        proj.pose.position.x = x;
        proj.pose.position.y = y;
        return proj;
    }
    double min_dist = std::numeric_limits<double>::max();
    for (const auto& p : corridor_path_) {
        double dx = x - p.pose.position.x;
        double dy = y - p.pose.position.y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            proj = p;
        }
    }
    return proj;
}

// 路径平滑
void ImprovedRRTPlanner::smoothPath(std::vector<geometry_msgs::PoseStamped>& plan) const {
    if (plan.size() < 4) return;

    std::vector<geometry_msgs::PoseStamped> smooth_plan;
    for (size_t i = 0; i < plan.size() - 1; ++i) {
        smooth_plan.push_back(plan[i]);
        int interp_num = 5;
        for (int j = 1; j < interp_num; ++j) {
            geometry_msgs::PoseStamped p;
            p.header = plan[i].header;
            double t = static_cast<double>(j) / interp_num;
            p.pose.position.x = plan[i].pose.position.x * (1 - t) + plan[i+1].pose.position.x * t;
            p.pose.position.y = plan[i].pose.position.y * (1 - t) + plan[i+1].pose.position.y * t;
            p.pose.orientation.w = 1.0;
            smooth_plan.push_back(p);
        }
    }
    smooth_plan.push_back(plan.back());
    plan = smooth_plan;
}

} // namespace global_planner
} // namespace amass