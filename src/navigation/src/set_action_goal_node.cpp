#include <cmath>
#include <string>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "set_action_goal_node");
  ros::NodeHandle pnh("~");

  double x = 0.0, y = 0.0, yaw = 0.0;
  std::string frame_id = "map";
  std::string server_name = "move_base";
  bool wait_result = true;
  double server_wait_sec = 10.0;
  double timeout_sec = 120.0;

  pnh.param("x", x, 0.0);
  pnh.param("y", y, 0.0);
  pnh.param("yaw", yaw, 0.0);
  pnh.param("frame_id", frame_id, std::string("map"));
  pnh.param("server_name", server_name, std::string("move_base"));
  pnh.param("wait_result", wait_result, true);
  pnh.param("server_wait_sec", server_wait_sec, 10.0);
  pnh.param("timeout_sec", timeout_sec, 120.0);

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac(server_name, true);

  ROS_INFO("Waiting for action server: %s", server_name.c_str());
  if (!ac.waitForServer(ros::Duration(server_wait_sec))) {
    ROS_ERROR("Action server not available.");
    return 1;
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.header.frame_id = frame_id;
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = std::sin(yaw * 0.5);
  goal.target_pose.pose.orientation.w = std::cos(yaw * 0.5);

  ac.sendGoal(goal);
  ROS_INFO("Sent action goal: frame=%s x=%.3f y=%.3f yaw=%.3f",
           frame_id.c_str(), x, y, yaw);

  if (wait_result) {
    bool ok = ac.waitForResult(ros::Duration(timeout_sec));
    if (!ok) {
      ROS_WARN("Wait result timeout.");
      return 2;
    }
    ROS_INFO("Action finished. state=%s", ac.getState().toString().c_str());
  }

  return 0;
}