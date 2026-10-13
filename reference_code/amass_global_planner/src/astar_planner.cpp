#include "astar_planner.h"
#include <costmap_2d/cost_values.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace amass {
namespace global_planner {

bool AStarPlanner::makePlan(const costmap_2d::Costmap2D* costmap,
                            const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            std::vector<geometry_msgs::PoseStamped>& plan) {
  plan.clear();
  if (!costmap) {
      ROS_ERROR("AStarPlanner: costmap is null!");
      return false;
  }

  // 起点、终点地图坐标：所在栅格下标
  int sx, sy, gx, gy;
  if (!worldToMap(costmap, start.pose.position.x, start.pose.position.y, sx, sy)) {
    ROS_ERROR("AStarPlanner: Start point is out of map bounds!");
    return false;
  }
  if (!worldToMap(costmap, goal.pose.position.x, goal.pose.position.y, gx, gy)) {
    ROS_ERROR("AStarPlanner: Goal point is out of map bounds!");
    return false;
  }

  // 检查起点终点可通行
  if (!isFree(costmap, sx, sy)) {
    ROS_ERROR("AStarPlanner: Start cell is not free!");
    return false;
  }
  if (!isFree(costmap, gx, gy)) {
    ROS_ERROR("AStarPlanner: Goal cell is not free!");
    return false;
  }

  // map地图宽度
  map_width = costmap->getSizeInCellsX();
  // 节点实例化; 结构体对象 
  Node start_node(sx, sy, 0, heuristic(sx, sy, gx, gy), -1);
  Node goal_node(gx, gy, 0, 0, -1);
  // 优先队列实现; f值小的优先, 即队列的顶部元素总是具有f最小的节点
  std::priority_queue<Node, std::vector<Node>, Compare> open_list;
  std::unordered_map<int, Node> closed_list;

  open_list.push(start_node);

  bool path_found = false;    // 标志位，发现可行路径为真
  Node current_node;

  while (!open_list.empty()) {
    current_node = open_list.top();
    open_list.pop();

    // 判断节点是否被访问过; 是则跳过该节点, 重新开始循环
    if (closed_list.find(current_node.id(map_width)) != closed_list.end())
      continue;
    closed_list[current_node.id(map_width)] = current_node;

    if (current_node == goal_node) {
      path_found = true;      // 遍历结束
      break;
    }

    // 遍历当前节点的所有邻居节点; 8邻域
    for (const auto& m : motions_) {
      int nx = current_node.x + m.first;      // 邻居节点的地图坐标
      int ny = current_node.y + m.second;
      if (nx < 0 || ny < 0 ||
        nx >= (int)costmap->getSizeInCellsX() ||
        ny >= (int)costmap->getSizeInCellsY())
        continue;
      if (!isFree(costmap, nx, ny))
        continue;
          
      // 计算从当前节点到其邻居节点的实际代价（g值）;
      // 判断邻居是否是正方向（上下左右），是则步长为1；否则为对角线，步长为√2
      double ng = current_node.g + ((m.first == 0 || m.second == 0) ? 1.0 : std::sqrt(2.0));
      double nh = heuristic(nx, ny, gx, gy);    // 启发成本

      Node neighbor_node(nx, ny, ng, nh, current_node.id(map_width));
      if (closed_list.find(neighbor_node.id(map_width)) != closed_list.end())
        continue;
      open_list.push(neighbor_node);
    }
  }

  if (!path_found) {
      ROS_WARN("AStarPlanner: No path found!");
      return false;
  }

  // 回溯路径
  reconstructPath(closed_list, current_node, plan, costmap, goal);

  return true;
}

void AStarPlanner::reconstructPath(const std::unordered_map<int, Node>& closed_list,
                                   Node current_node,
                                   std::vector<geometry_msgs::PoseStamped>& plan,
                                   const costmap_2d::Costmap2D* costmap,
                                   const geometry_msgs::PoseStamped& goal) const {

  std::vector<geometry_msgs::PoseStamped> rplan;  // 反向路径
  double wx, wy;
  geometry_msgs::PoseStamped pose;
  pose.header = goal.header; // 保持目标点的坐标系

  while (true) {

    // costmap->mapToWorld(current_node.x, current_node.y, wx, wy);
    mapToWorld(costmap, current_node.x, current_node.y, wx, wy);

    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    rplan.push_back(pose);
    if (current_node.pid == -1) break;
    auto it = closed_list.find(current_node.pid); // 查找父节点
    if (it == closed_list.end()) break;

    // std::pair<const Key, T>类型的对象包含两个成员：.first：键（key）second：值（value）
    current_node = it->second;                    
  }
  // 路径反转
  plan.assign(rplan.rbegin(), rplan.rend());
  // 确保终点精确
  plan.back() = goal;
}

} // namespace global_planner
} // namespace amass