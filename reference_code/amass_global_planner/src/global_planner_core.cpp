#include "global_planner_core.h"
#include "astar_planner.h"
#include "pso_planner.h"
#include "rrt_planner.h"
#include "improved_rrt_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(amass::global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace amass {
namespace global_planner {

GlobalPlanner::GlobalPlanner()
    : initialized_(false), costmap_ros_(nullptr), costmap_(nullptr), 
    planner_(nullptr), allow_unknown_(false) {}

GlobalPlanner::~GlobalPlanner() {
    if (planner_) delete planner_;  // 析构函数删除对象，释放内存
}
/*
  参数：
      name：该规划器插件的命名空间前缀；
      costmap_ros：指向 ROS 下的代价地图对象，用于获取环境的栅格地图（障碍物、自由空间等信息）
*/
GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : GlobalPlanner() {
    initialize(name, costmap_ros);  // 函数重载
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (initialized_) {
        ROS_INFO("amass_global_planner has already been initialized.");
        return;
    }
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    global_frame_ = costmap_ros_->getGlobalFrameID();

    ros::NodeHandle private_nh("~/" + name);
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1, true); 
    private_nh.param<std::string>("planner_type", planner_type_, "astar");
    private_nh.param("use_dijkstra", use_dijkstra_, false);
    private_nh.param("allow_unknown", allow_unknown_, false);
    private_nh.param("planner_window_x", planner_window_x_, 0.0);
    private_nh.param("planner_window_y", planner_window_y_, 0.0);
    private_nh.param("default_tolerance", default_tolerance_, 0.0);
    private_nh.param("outline_map", outline_map_, true);


    if (planner_) delete planner_;         // 释放原有的算法对象，防止内存泄漏
    if (planner_type_ == "astar") {
      planner_ = new AStarPlanner();
      planner_->setDijkstra(use_dijkstra_);
      if(use_dijkstra_){
        ROS_INFO("amass_global_planner: Using Dijkstra algorithm as global planner.");
      }
      else{
        ROS_INFO("amass_global_planner: Using Astar algorithm as global planner.");
      }
    } 
    else if(planner_type_ == "pso"){
      planner_ = new PSOPlanner();
      ROS_INFO("amass_global_planner: Using PSO algorithm as global planner.");
    }
    else if(planner_type_ == "rrt"){
      planner_ = new RRTPlanner();
      ROS_INFO("amass_global_planner: Using RRT algorithm as global planner.");
    }
    else if(planner_type_ == "improved_rrt"){
        planner_ = new ImprovedRRTPlanner();
        ROS_INFO("amass_global_planner: Using Improved RRT algorithm as global planner.");
    }
    else {
      planner_ = new AStarPlanner();
      ROS_INFO("amass_global_planner: Using Astar algorithm as default global planner.");
    }

    planner_->setAllowUnknown(allow_unknown_);
    initialized_ = true;
    ROS_INFO("amass_global_planner initialized.");
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
}



bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal,
                             double tolerance,
                             std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("amass_global_planner has not been initialized.");
        return false;
    }
    plan.clear();
    if (!planner_) {
        ROS_INFO("amass_global_planner: no global planner selected.");
        return false;
    }

    bool result = planner_->makePlan(costmap_, start, goal, plan);

    // 发布路径
    publishPlan(plan);
    return result;
}

/*
    功能：全局路径规划path发布，在rviz中显示
*/

void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = global_frame_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}



/**
 * @brief Check the validity of (wx, wy) 坐标合法性检查-是否在地图内
 * @param wx world map x
 * @param wy world map y
 * @param mx costmap x
 * @param my costmap y
 * @return flag true if the position is valid
 */
// bool AbstractAlgorithm::validityCheck(double wx, double wy, double& mx, double& my) {
//   if (!world2Map(wx, wy, mx, my)) {
//     ROS_ERROR("The robot's position is off the global costmap. Planning will always fail");
//     return false;
//   }
//   return true;
// }

/* 
  功能：世界坐标转地图坐标，返回是否有效
  参数：
      costmap：代价地图指针
      (wx, wy)：世界坐标系(map)下节点坐标，对应真实物理坐标(米)
      (mx, my)：节点所在栅格下标，例如(10,1)-第10行第1列的栅格
  返回值：true 表示转换后坐标在地图范围内
*/
bool AbstractAlgorithm::worldToMap(const costmap_2d::Costmap2D* costmap, double wx, double wy, int& mx, int& my) const {
  double origin_x = costmap->getOriginX();
  double origin_y = costmap->getOriginY();
  double resolution = costmap->getResolution();         // 地图分辨率
  mx = static_cast<int>((wx - origin_x) / resolution);  
  my = static_cast<int>((wy - origin_y) / resolution);
  return mx >= 0 && my >= 0 && mx < (int)costmap->getSizeInCellsX() && my < (int)costmap->getSizeInCellsY();
}

void AbstractAlgorithm::mapToWorld(const costmap_2d::Costmap2D* costmap, int mx, int my, double& wx, double& wy) const {
    wx = costmap->getOriginX() + (mx + 0.5) * costmap->getResolution();
    wy = costmap->getOriginY() + (my + 0.5) * costmap->getResolution();
}

/*
  功能：判断地图格子(mx, my)是否可通行
  参数：
      costmap：代价地图指针
      (mx, my)：节点所在栅格下标
  返回值：true 表示该格子可通行（非障碍物且有已知信息）
*/
bool AbstractAlgorithm::isFree(const costmap_2d::Costmap2D* costmap, int mx, int my) const {
    // 边界检查
    if (mx < 0 || my < 0 || mx >= (int)costmap->getSizeInCellsX() || my >= (int)costmap->getSizeInCellsY())
        return false;

    unsigned char cost = costmap->getCost(mx, my);
    // 只允许完全自由空间通过
    if (cost == costmap_2d::FREE_SPACE || (cost == costmap_2d::NO_INFORMATION && allow_unknown_))
        return true;
    return false;
    
    // 如果允许膨胀区通过，可以放宽为：
    // return cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE && cost != costmap_2d::NO_INFORMATION;
}

} // namespace global_planner
} // namespace amass

