#ifndef AMASS_ASTAR_PLANNER_H
#define AMASS_ASTAR_PLANNER_H

#include "global_planner_core.h"
#include <queue>
#include <vector>
#include <cmath>
#include <unordered_map>

namespace amass {
namespace global_planner {

// A*算法实现类，继承全局规划器算法基类
class AStarPlanner : public AbstractAlgorithm {
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  // 路径规划主接口
  bool makePlan(const costmap_2d::Costmap2D* costmap,
                const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
  // 节点结构体，表示地图上的一个格子
  struct Node {
    int x, y;           // 地图坐标; 节点所在栅格下标
    double g, h;        // g：实际成本；h：启发成本
    int pid;            // 父节点索引（用于回溯路径）

    // 获取节点唯一索引
    int id(int map_width) const { return y * map_width + x; }
    // 计算总代价 f = g + h
    double f() const { return g + h; }

    // 构造函数
    Node(int x_=0, int y_=0, double g_=0, double h_=0, int pid_=-1)
        : x(x_), y(y_), g(g_), h(h_), pid(pid_){}

    // 判断节点是否相等（坐标相同）
    bool operator==(const Node& other) const { return x == other.x && y == other.y; }
  };

  // 优先队列比较器，f值小的优先
  struct Compare {
    bool operator()(const Node& a, const Node& b) const {
        return a.f() > b.f();
    }
  };

  // 8邻域运动集合; {x方向增量, y方向增量}; 以栅格为单位
  const std::vector<std::pair<int, int>> motions_ = {
    {0, 1}, {1, 0}, {0, -1}, {-1, 0},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };

  // 启发函数，欧氏距离
  double heuristic(int x1, int y1, int x2, int y2) const {
    if(use_dijkstra_) return 0.0;
    return std::hypot(x1 - x2, y1 - y2);
  }

  // 路径回溯，生成最终路径
  void reconstructPath(const std::unordered_map<int, Node>& closed,
                        Node current,
                        std::vector<geometry_msgs::PoseStamped>& plan,
                        const costmap_2d::Costmap2D* costmap,
                        const geometry_msgs::PoseStamped& goal) const;
};

} // namespace global_planner
} // namespace amass

#endif