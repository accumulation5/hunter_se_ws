#include <string>
#include <cmath>
#include <nav2_util/node_utils.hpp>
#include "nav2_amass_planner/straightline_planner.hpp"

namespace nav2_amass_planner
{
  /* 插件初始化，参数配置 */
  void StraightLine::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    // 参数拷贝
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros;
    global_frame_ = costmap_->getGlobalFrameID();

    // 向参数服务器注册参数
    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".map_resolution", rclcpp::ParameterValue(0.05));
    node_->get_parameter(name_ + ".map_resolution", map_resolution_);  // 从参数服务器获取参数
  }
  
  /* 插件资源清除 */
  void StraightLine::cleanup(){
    RCLCPP_INFO(node_->get_logger(), "CleaningUp plugin %s of type nav2_amass_planner",
      name_.c_str());
  } 
  
  /* 激活插件 */
  void StraightLine::activate(){
    RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type nav2_amass_planner",
      name_.c_str());
  }
  
  /* 失能插件 */
  void StraightLine::deactivate(){
    RCLCPP_INFO(node_->get_logger(), "DeActivating plugin %s of type nav2_amass_planner",
      name_.c_str());
  }

  nav_msgs::msg::Path StraightLine::createPlan(
    const geometry_msgs::msg::PoseStamped &start,
		const geometry_msgs::msg::PoseStamped &goal)
  {
    nav_msgs::msg::Path global_path_;

    // 起点终点可行性检查

    global_path_.poses.clear();
    global_path_.header.stamp = node_->now();
    global_path_.header.frame_id = global_frame_;
    // 根据当前插值分辨率的值计算循环次数
    // int total_num_of_loop = std::hypot(
    //   goal.pose.position.x - start.pose.position.x, 
    //   goal.pose.position.y - start.pose.position.y) / map_resolution_;
    int x_num_loop = std::abs(goal.pose.position.x - start.pose.position.x) / map_resolution_;
    int y_num_loop = std::abs(goal.pose.position.y - start.pose.position.y) / map_resolution_;
    double x_increment = (goal.pose.position.x - start.pose.position.x) / x_num_loop;
    double y_increment = (goal.pose.position.y - start.pose.position.y) / y_num_loop;
    
    geometry_msgs::msg::PoseStamped pose_;
    for(int i=0; i < x_num_loop; i++)
    {
      pose_.header.frame_id = global_frame_;
      pose_.header.stamp = node_->now();
      pose_.pose.position.x = start.pose.position.x + i*x_increment;
      pose_.pose.position.y = start.pose.position.y;
      pose_.pose.position.z = 0.0;
      pose_.pose.orientation.x = 0.0;
      pose_.pose.orientation.y = 0.0;
      pose_.pose.orientation.z = 0.0;
      pose_.pose.orientation.w = 1.0;
      global_path_.poses.push_back(pose_);
    }
    double last_pose_x = pose_.pose.position.x;
    for(int i=0; i < y_num_loop; i++)
    {
      pose_.header.frame_id = global_frame_;
      pose_.header.stamp = node_->now();
      pose_.pose.position.x = last_pose_x;
      pose_.pose.position.y = start.pose.position.y + i*y_increment;
      pose_.pose.position.z = 0.0;
      pose_.pose.orientation.x = 0.0;
      pose_.pose.orientation.y = 0.0;
      pose_.pose.orientation.z = 0.0;
      pose_.pose.orientation.w = 1.0;
      global_path_.poses.push_back(pose_);
    }

    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header.frame_id = global_frame_;
    goal_pose.header.stamp = node_->now();
    global_path_.poses.push_back(goal_pose);
      
    return global_path_;

  }

}   // namespace nav2_amass_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_amass_planner::StraightLine, nav2_core::GlobalPlanner)