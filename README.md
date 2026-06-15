该工作空间基于hunter_se移动底盘，用于空地协同比赛仿真与实车验证

# 环境说明

- ubuntu20.04

- ros-noetic


# 文件系统

```
├── drivers：真实场景下，物理硬件的驱动文件
│   ├── GNSS-RTK：G70模块
│   ├── hunter_ros：hunter_se底盘控制
│   ├── lidar-N10P：N10P单线雷达
│   └── robot_localization：多源融合定位功能包
├── hunter_se_simulation：仿真相关文件夹
│   ├── drivers：仿真相关驱动文件/功能包/插件，用于在gazebo中模拟物理硬件
│   ├── hunter_se_description：底盘及其他硬件的描述文件
│   ├── hunter_se_gazebo：gazebo相关功能包，包含world、models等文件
│   └── hunter_se_navigation：仿真场景下，基础导航功能包
├── navigation：真实场景下，基础导航功能包
└── ugv_sdk：底盘相关接口定义
```



# 仿真

**坐标系设置**

```
odom_frame_id：odom
base_frame_id：base_footprint
global_frame_id：map
lidar：lidar
cmd_vel话题名称：/ackermann_steering_controller/cmd_vel
```

**运行**

```
编译
cd hunter_ws
catkin clean -b -d
source ./devel/setup.bash

rviz展示模型
roslaunch hunter_se_description display.launch

gazebo仿真
roslaunch hunter_se_gazebo hunter_se_gazebo.launch run_rviz:=true

导航仿真流程
0. 刷新环境变量
	source ./devel/setup.bash
1. 运行hunter_se_gazebo.launch，启动gazebo与rviz，确保各类传感器感知、坐标变换、话题订阅、运动控制等功能正常
	roslaunch hunter_se_gazebo gazebo.launch run_rviz:=true
2. 运行slam_gmapping.launch，同时运行键盘控制节点，控制机器人运动建图
	roslaunch hunter_se_navigation slam_gmapping.launch
	rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/ackermann_steering_controller/cmd_vel 
3. 运行map_saver.launch，保存地图至指定路径
	roslaunch hunter_se_navigation map_saver.launch
4. 终止所有进程，新建终端，启动所有仿真环境、组件、话题等内容，通过Rviz工具栏的 2D Nav Goal设置目标点实现导航
	roslaunch hunter_se_navigation hunter_se_navigation.launch
6. 运行 set_action_goal.launch，自行输入目标点坐标值，实现导航
	roslaunch hunter_se_navigation set_action_goal.launch x:=1.0 y:=2.0 yaw:=0.0 frame_id:=map
说明：
	1. set_action_goal.launch
        - 目标点坐标需在map坐标系下，且在合理范围，而非障碍物、地图外等非法区域、
        - set_action_goal.launch 启动 set_action_goal_node 节点（动作通信）
        通过 actionlib 客户端直接发送 MoveBaseGoal 发布目标点坐标，由move_base节点订阅，实现路径规划 
```





# 真实场景

**坐标系设置**

```
odom_frame_id：odom
base_frame_id：base_footprint
global_frame_id：map
lidar：lidar
cmd_vel指令：cmd_vel
```



**键盘控制**

```
CAN通信
1. 使能 gs_usb 内核模块
sudo modprobe gs_usb
2. 设置 500k 波特率和使能 can-to-usb 适配器
sudo ip link set can0 up type can bitrate 500000
3. 底盘节点启动	
roslaunch hunter_bringup hunter_robot_base.launch
4. 键盘控制（指定 线速度speed=0.3m/s 角速度turn=0.3rad/s）
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.3 _turn:=0.3 
```



**运行**

```
0. 使能 gs_usb 内核模块；设置 500k 波特率和使能 can-to-usb 适配器
	sudo modprobe gs_usb
	sudo ip link set can0 up type can bitrate 500000
	candump can0 （监听can0通道数据）
    ls /dev/tty* 2>/dev/null | grep -E "(ACM|USB|CH343)"
    	查看各传感器对应的端口名称，在对应的驱动文件处修改！
1. 运行hunter_start.launch，启动rviz，检查底盘odom数据、TF树、传感器数据等是否正常
	source devel/setup.bash
	roslaunch navigation hunter_start.launch
2. 运行slam_gampping.launch，rviz坐标系选择map
	source devel/setup.bash
	roslaunch navigation slam_gmapping.launch
3. 键盘控制小车运动，完成slam建图
	rosrun teleop_twist_keyboard teleop_twist_keyboard.py
4. 运行 map_saver.launch，保存全局地图
	source devel/setup.bash
	roslaunch navigation map_saver.launch 
5. 运行navigation.launch与hunter_start.launch，完成amcl定位与move_base导航
	source devel/setup.bash
	roslaunch navigation navigation.launch
```



# 分布式通信

**环境配置**

ROS1采用分布式通信架构，两台计算机可以通过网络实现通信

**配置要求**

网络连接：两台计算机需要在**同一个局域网**内，通常通过网线或Wi-Fi连接(有线、无线)

> 与内网的服务器/计算机通信，需要通过通过公网IP和端口转发访问

**配置步骤**

```
1. 对主从机分别设置 静态固定IP ，确保处于 同一局域网段，并配置主从机环境变量
	静态ip设置：ubuntu打开设置-wifi，在已连接的wifi处，修改IPv4方式为手动，设置静态ip、子网掩码、网关
	IP地址查看: ifconfig
    计算机名称查看: hostname
2. 配置文件修改
	分别修改不同计算机的 /etc/hosts 文件，在该文件中加入对方的IP地址和计算机名:
    主机端:
    	从机的IP    从机计算机名
    从机端:
    	主机的IP    主机计算机名
    设置完毕，可以通过 ping 命令测试网络通信是否正常
    	主机端：ping 从机的IP（从机端反之）
3.配置主机IP
    配置主机的 IP 地址
    ~/.bashrc 追加
        export ROS_MASTER_URI=http://主机IP:11311
        export ROS_HOSTNAME=主机IP
4.配置从机IP
    配置从机的 IP 地址，从机可以有多台，每台都做如下设置:
    ~/.bashrc 追加
        export ROS_MASTER_URI=http://主机IP:11311
        export ROS_HOSTNAME=从机IP
5. 测试
	1.主机启动 roscore(必须)
		roscore
    2.主机启动订阅节点，从机启动发布节点，测试通信是否正常
    	rosrun turtlesim turtlesim_node
    3.反向测试，主机启动发布节点，从机启动订阅节点，测试通信是否正常
    	rostopic list
    	rostopic info /turtle1/pose
    	...
```

**运行**

**工控机**（主机）**PC**（从机）

> 工控机断电前，首先使用ssh远程关机：sudo shutdown now

```
1. rviz静态展示
	主机
	rosluanch hunte_se_description display.launch run_rviz:=false
	从机
	rviz -d /home/amass/hunter_ws/src/hunter_se_simulation/hunter_se_description/config/rviz.rviz
	rviz(通过菜单栏加载config文件)
	说明：由于从机无法从urdf路径中加载对应的stl文件，因此，无法显示robotmodel的mesh皮肤；测试时仅使用tf查看底盘位置即可
2. gazebo仿真
	主机
	roslaunch hunter_se_gazebo hunter_se_gazebo.launch run_gazebo:=true run_rviz:=false gazebo_gui:=false
	从机
	gzclient
	说明：失败
3. 真实场景导航（主机）
    0. 使能 gs_usb 内核模块；设置 500k 波特率和使能 can-to-usb 适配器
        sudo modprobe gs_usb
        sudo ip link set can0 up type can bitrate 500000
        candump can0 （监听can0通道数据）
    	ls /dev/tty* 2>/dev/null | grep -E "(ACM|USB|CH343)"
    		查看各传感器对应的端口名称，在对应的驱动文件处修改！
    1. 运行hunter_start.launch，启动rviz，检查底盘odom数据、TF树、传感器数据等是否正常
        source devel/setup.bash
        roslaunch navigation hunter_start.launch run_rviz:=false
    2. 运行slam_gampping.launch，rviz坐标系选择map
        source devel/setup.bash
        roslaunch navigation slam_gmapping.launch
    3. 键盘控制小车运动，完成slam建图
        rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    4. 运行 map_saver.launch，保存全局地图
        source devel/setup.bash
        roslaunch navigation map_saver.launch 
    5. 运行navigation.launch与hunter_start.launch，完成amcl定位与move_base导航
        source devel/setup.bash
        roslaunch navigation navigation.launch run_rviz:=false 
```



