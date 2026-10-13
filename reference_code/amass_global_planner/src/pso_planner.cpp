#include "pso_planner.h"
#include <ros/ros.h>
#include <cmath>
#include <limits>

namespace amass {
namespace global_planner {

PSOPlanner::PSOPlanner()
    : particle_num_(30), iter_num_(100), path_points_num_(10),
      w_(0.5), c1_(1.5), c2_(1.5) {
    // 可根据地图实际范围调整
    map_min_x_ = 0.0; map_min_y_ = 0.0; map_max_x_ = 20.0; map_max_y_ = 20.0;
}

bool PSOPlanner::makePlan(const costmap_2d::Costmap2D* costmap,
                          const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan) {
    plan.clear();
    std::vector<Particle> particles(particle_num_);
    initParticles(particles, start, goal);

    Particle gbest = particles[0];
    for (auto& p : particles) {
        p.fitness = fitness(p.path, costmap);
        p.best_path = p.path;
        p.best_fitness = p.fitness;
        if (p.fitness < gbest.fitness) gbest = p;
    }

    std::uniform_real_distribution<double> dist(0, 1);

    for (int iter = 0; iter < iter_num_; ++iter) {
        for (auto& p : particles) {
            for (size_t i = 1; i < p.path.size() - 1; ++i) { // 路径首尾点不动
                for (int d = 0; d < 2; ++d) {
                    double& v = (d == 0) ? p.velocity[i].first : p.velocity[i].second;
                    double pb = (d == 0) ? p.best_path[i].first : p.best_path[i].second;
                    double pa = (d == 0) ? p.path[i].first : p.path[i].second;
                    double gb = (d == 0) ? gbest.path[i].first : gbest.path[i].second;

                    double rp = dist(rng_), rg = dist(rng_);
                    v = w_ * v + c1_ * rp * (pb - pa) + c2_ * rg * (gb - pa);

                    if (d == 0) p.path[i].first += v;
                    else        p.path[i].second += v;

                    // 限制在地图范围内
                    if (d == 0) p.path[i].first = std::min(std::max(p.path[i].first, map_min_x_), map_max_x_);
                    else        p.path[i].second = std::min(std::max(p.path[i].second, map_min_y_), map_max_y_);
                }
            }
            p.fitness = fitness(p.path, costmap);
            if (p.fitness < p.best_fitness) {
                p.best_fitness = p.fitness;
                p.best_path = p.path;
            }
            if (p.fitness < gbest.fitness) {
                gbest = p;
            }
        }
    }

    // 输出最优路径
    for (const auto& pt : gbest.path) {
        geometry_msgs::PoseStamped pose = start;
        pose.pose.position.x = pt.first;
        pose.pose.position.y = pt.second;
        plan.push_back(pose);
    }
    // 确保终点
    plan.back() = goal;
    return true;
}

void PSOPlanner::initParticles(std::vector<Particle>& particles,
                              const geometry_msgs::PoseStamped& start,
                              const geometry_msgs::PoseStamped& goal) {
    std::uniform_real_distribution<double> dist_x(map_min_x_, map_max_x_);
    std::uniform_real_distribution<double> dist_y(map_min_y_, map_max_y_);
    for (auto& p : particles) {
        p.path.resize(path_points_num_);
        p.velocity.resize(path_points_num_, {0.0, 0.0});
        // 起点
        p.path[0] = {start.pose.position.x, start.pose.position.y};
        // 终点
        p.path[path_points_num_ - 1] = {goal.pose.position.x, goal.pose.position.y};
        // 中间点随机初始化
        for (int i = 1; i < path_points_num_ - 1; ++i) {
            p.path[i] = {dist_x(rng_), dist_y(rng_)};
        }
    }
}

double PSOPlanner::fitness(const std::vector<std::pair<double, double>>& path, const costmap_2d::Costmap2D* costmap) {
    double cost = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i].first - path[i-1].first;
        double dy = path[i].second - path[i-1].second;
        cost += std::hypot(dx, dy); // 路径长度
        // 惩罚穿越障碍物
        int mx, my;
        if (worldToMap(costmap, path[i].first, path[i].second, mx, my)) {
            unsigned char c = costmap->getCost(mx, my);
            if (c >= costmap_2d::LETHAL_OBSTACLE)
                cost += 1000.0; // 大惩罚
        } else {
            cost += 1000.0;
        }
    }
    return cost;
}

} // namespace global_planner
} // namespace amass