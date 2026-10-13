#ifndef AMASS_IMPROVED_RRT_PLANNER_H
#define AMASS_IMPROVED_RRT_PLANNER_H

#include "global_planner_core.h"
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <random>

namespace amass {
namespace global_planner {

struct ImprovedRRTNode {
    double x, y;
    int parent_id;
    double cost;
    ImprovedRRTNode(double x_=0, double y_=0, int parent_id_=-1, double cost_=0)
        : x(x_), y(y_), parent_id(parent_id_), cost(cost_) {}
};

class ImprovedRRTPlanner : public AbstractAlgorithm {
public:
    ImprovedRRTPlanner();
    bool makePlan(const costmap_2d::Costmap2D* costmap,
                  const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

    // 设置走廊轨迹
    void setCorridorPath(const std::vector<geometry_msgs::PoseStamped>& path);

private:
    // 参数
    double step_size_;
    double goal_tolerance_;
    double goal_bias_prob_;
    int max_iterations_;
    double corridor_width_;
    double alpha_; // 引导权重

    std::vector<ImprovedRRTNode> tree_;
    std::vector<geometry_msgs::PoseStamped> corridor_path_;
    std::default_random_engine rng_;

    bool isValidSample(const costmap_2d::Costmap2D* costmap, double x, double y) const;
    bool inCorridor(double x, double y) const;
    int nearestNode(double x, double y) const;
    bool collisionCheck(const costmap_2d::Costmap2D* costmap, const ImprovedRRTNode& from, const ImprovedRRTNode& to) const;
    geometry_msgs::PoseStamped getProjectionOnCenterLine(double x, double y) const;
    void smoothPath(std::vector<geometry_msgs::PoseStamped>& plan) const;
};

} // namespace global_planner
} // namespace amass

#endif