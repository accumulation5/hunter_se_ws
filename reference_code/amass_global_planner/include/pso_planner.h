#ifndef AMASS_PSO_GLOBAL_PLANNER_H
#define AMASS_PSO_GLOBAL_PLANNER_H

#include "global_planner_core.h"
#include <vector>
#include <random>

namespace amass {
namespace global_planner {

struct Particle {
    std::vector<std::pair<double, double>> path; // 路径点
    double fitness;
    std::vector<std::pair<double, double>> velocity;
    std::vector<std::pair<double, double>> best_path;
    double best_fitness;
};

class PSOPlanner : public AbstractAlgorithm {
public:
    PSOPlanner();
    bool makePlan(const costmap_2d::Costmap2D* costmap,
                  const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    int particle_num_;
    int iter_num_;
    int path_points_num_;
    double w_, c1_, c2_;
    double map_min_x_, map_min_y_, map_max_x_, map_max_y_;

    double fitness(const std::vector<std::pair<double, double>>& path, const costmap_2d::Costmap2D* costmap);
    void initParticles(std::vector<Particle>& particles,
                      const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal);
    std::default_random_engine rng_;
};

} // namespace global_planner
} // namespace amass

#endif