#ifndef AMASS_RRT_PLANNER_H
#define AMASS_RRT_PLANNER_H

#include "global_planner_core.h"
#include <vector>
#include <unordered_map>
#include <random>
#include <geometry_msgs/PoseStamped.h>

namespace amass {
namespace global_planner {

struct RRTNode {
    double x, y;
    int parent_id;
    int id;
    double cost;
    RRTNode(double x_=0, double y_=0, int parent_id_=-1, int id_=-1, double cost_=0)
        : x(x_), y(y_), parent_id(parent_id_), id(id_), cost(cost_) {}
};

class RRTPlanner : public AbstractAlgorithm {
public:
    RRTPlanner();
    bool makePlan(const costmap_2d::Costmap2D* costmap,
                  const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    // 参数
    int max_iter_;
    double step_size_;
    double goal_sample_rate_; // [0,1]
    double goal_tolerance_;   // 终点判定距离
    double map_min_x_, map_min_y_, map_max_x_, map_max_y_;

    std::default_random_engine rng_;

    int toId(int mx, int my, int map_width) const { return my * map_width + mx; }
    void toGrid(double wx, double wy, const costmap_2d::Costmap2D* costmap, int& mx, int& my) const;
    bool isCollisionFree(const RRTNode& n1, const RRTNode& n2, const costmap_2d::Costmap2D* costmap) const;
    double distance(const RRTNode& n1, const RRTNode& n2) const;
    RRTNode sampleRandomNode(const costmap_2d::Costmap2D* costmap, const geometry_msgs::PoseStamped& goal);
    int getNearestNodeId(const std::unordered_map<int, RRTNode>& nodes, const RRTNode& node, int map_width) const;
    void backtracePath(const std::unordered_map<int, RRTNode>& nodes, int goal_id,
                       std::vector<geometry_msgs::PoseStamped>& plan,
                       const costmap_2d::Costmap2D* costmap, const geometry_msgs::PoseStamped& goal) const;
};

} // namespace global_planner
} // namespace amass

#endif