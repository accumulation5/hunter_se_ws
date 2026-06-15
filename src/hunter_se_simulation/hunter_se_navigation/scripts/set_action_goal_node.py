#!/usr/bin/env python3

'''
    使用action通信, 发送目标点到move_base, 等待结果
    用 actionlib 客户端直接发送 MoveBaseGoal
    move_base_msgs/MoveBaseActionGoal
'''

import rospy
import actionlib
import tf.transformations as tft
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def main():
    rospy.init_node("set_action_goal_node")

    x = rospy.get_param("~x", 0.0)
    y = rospy.get_param("~y", 0.0)
    yaw = rospy.get_param("~yaw", 0.0)  # rad
    frame_id = rospy.get_param("~frame_id", "map")
    server_name = rospy.get_param("~server_name", "move_base")
    wait_result = rospy.get_param("~wait_result", True)
    timeout_sec = rospy.get_param("~timeout_sec", 120.0)

    client = actionlib.SimpleActionClient(server_name, MoveBaseAction)
    rospy.loginfo("Waiting for action server: %s", server_name)
    if not client.wait_for_server(rospy.Duration(10.0)):
        rospy.logerr("move_base action server not available.")
        return

    q = tft.quaternion_from_euler(0.0, 0.0, yaw)

    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    client.send_goal(goal)
    rospy.loginfo("Sent action goal: frame=%s x=%.3f y=%.3f yaw=%.3f",
                  frame_id, x, y, yaw)

    if wait_result:
        ok = client.wait_for_result(rospy.Duration(timeout_sec))
        if not ok:
            rospy.logwarn("Wait result timeout.")
            return
        state = client.get_state()
        rospy.loginfo("Action finished, state=%d", state)

if __name__ == "__main__":
    main()