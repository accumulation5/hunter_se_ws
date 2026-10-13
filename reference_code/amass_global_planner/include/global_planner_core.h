#ifndef AMASS_GLOBAL_PLANNER_CORE_H
#define AMASS_GLOBAL_PLANNER_CORE_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <string>
#include <nav_msgs/Path.h>


namespace amass {
namespace global_planner {

  // 抽象类，所有其他算法规划器均继承此类，负责算法接口和通用函数实现
  class AbstractAlgorithm {
    public:
      virtual ~AbstractAlgorithm() = default;
      virtual bool makePlan(const costmap_2d::Costmap2D* costmap,
                          const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan) = 0;
      /* 
        定义：世界坐标转地图坐标，返回是否有效
        参数：
          costmap：代价地图指针
          (wx, wy)：世界坐标系(map)下节点坐标，对应真实物理坐标
          (mx, my)：节点所在栅格下标，例如(10,1)-第10行第1列的栅格
      */
      bool worldToMap(const costmap_2d::Costmap2D* costmap, double wx, double wy, int& mx, int& my) const;
      void mapToWorld(const costmap_2d::Costmap2D* costmap, int mx, int my, double& wx, double& wy) const;
      
      /**
       * @brief Check the validity of (wx, wy) 坐标合法性检查-是否在地图内
       * @param wx world map x
       * @param wy world map y
       * @param mx costmap x
       * @param my costmap y
       * @return flag true if the position is valid 
       */
      bool validityCheck(double wx, double wy, double& mx, double& my);

      /*
        功能：判断地图格子(mx, my)是否可通行
        参数：
            costmap：代价地图指针
            (mx, my)：节点所在栅格下标
        返回值：true 表示该格子可通行（非障碍物且有已知信息）
      */
      bool isFree(const costmap_2d::Costmap2D* costmap, int mx, int my) const;
      // 设置允许未知区域通信参数
      void setAllowUnknown(bool allow_unknown) { allow_unknown_ = allow_unknown; };

      void setDijkstra(bool use_dijkstra) {use_dijkstra_ = use_dijkstra; };
    
    protected:
      int map_width, map_height;            // 地图大小
      bool allow_unknown_ = false;         // 是否允许未知区域
      bool use_dijkstra_ = false;            // 是否使用dijkstra算法
  };

  // 全局路径规划器封装类，负责与ROS框架对接、插件注册、参数管理和算法切换
  class GlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:
      GlobalPlanner();
      
      GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      
      ~GlobalPlanner();

      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

      bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

      // 带容差的重载
      bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  double tolerance,
                  std::vector<geometry_msgs::PoseStamped>& plan);
      // 路径发布
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

    protected:
      double planner_window_x_, planner_window_y_, default_tolerance_;
      bool initialized_, allow_unknown_, outline_map_, use_dijkstra_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D* costmap_;
      std::string global_frame_;
      std::string planner_type_; // "dijkstra" or "astar"
      AbstractAlgorithm* planner_;

    private:
      ros::Publisher plan_pub_;
  };

  

} // namespace global_planner

} // namespace amass

#endif // AMASS_GLOBAL_PLANNER_CORE_H