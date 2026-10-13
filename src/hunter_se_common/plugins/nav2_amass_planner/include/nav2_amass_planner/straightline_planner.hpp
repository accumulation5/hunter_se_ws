#ifndef NAV2_AMASS_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_AMASS_PLANNER__STRAIGHT_LINE_PLANNER_HPP_


#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_core/global_planner.hpp>



namespace nav2_amass_planner
{

  class StraightLine : public nav2_core::GlobalPlanner{
  
    public:
      StraightLine() = default;
      ~StraightLine() = default;

      // 插件初始化配置
      void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer>tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)override;
      
      // 插件清除：清理为规划器创建的资源
      void cleanup()override;
      
      // 插件激活
      void activate()override;

      // 插件失能
      void deactivate()override;

			// 给定起点和目标点，创建可行路径
			nav_msgs::msg::Path createPlan(
				const geometry_msgs::msg::PoseStamped &start,
				const geometry_msgs::msg::PoseStamped &goal
			)override;
        

    private:
        // node 共享指针
        nav2_util::LifecycleNode::SharedPtr node_;
				// TF buffer
				std::shared_ptr<tf2_ros::Buffer> tf_;
        // 插件名称
        std::string name_;
        // 代价地图指针
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
        // 代价地图全局坐标系名称
        std::string global_frame_;

        // 地图分辨率
        double map_resolution_;
  };


} // namespace nav2_amass_planner


#endif