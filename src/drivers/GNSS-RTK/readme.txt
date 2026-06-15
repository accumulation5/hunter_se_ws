一、编译相关：
编译时可能遇到的报错：
1、Could not find a package configuration file provided by "catkin_virtualenv"

解决办法：sudo apt install ros-noetic-catkin-virtualenv

2、ERROR: launchpadlib 1.10.13 requires testresources, which is not installed.
解决办法：sudo apt install python3-testresources

3、ERROR: ModuleNotFoundError: No module named 'serial'.
解决办法：sudo apt install python3-pip
pip3 install serial
pip3 install pyserial

4、ERROR:ModuleNotFoundError: No module named 'pyproj'
解决办法：pip install pyproj
安装依赖：
sudo pip3 install transforms3d
sudo apt install ros-noetic-tf-transformations


或一键安装依赖：
rosdepc install --from-paths src --ignore-src -r -y

---------------------------------------------------------------------------------------------

二、运行相关：

运行设备别名规则文件:
sudo sh wheeltec_gnss.sh


使用NMEA协议解析WHEELTEC G60/G70：
roslaunch wheeltec_gps_driver wheeltec_nmea_driver.launch

使用NMEA协议解析WHEELTEC G90：
roslaunch wheeltec_gps_driver wheeltec_dual_rtk_driver_nmea.launch

--gps topic: /gps/fix
--gps frame_id: navsat_link

在RVIZ中记录经纬度轨迹(WHEELTEC G60/G70)：
roslaunch wheeltec_gps_driver nmea_gps_path.launch

在RVIZ中记录经纬度轨迹(WHEELTEC 90)：
roslaunch wheeltec_gps_driver nmea_gps_path_dualrtk.launch

---------------------------------------------------------------------------------------------

使用UBLOX协议解析WHEELTEC G70模块(仅WHEELTEC G70)：
roslaunch wheeltec_gps_driver wheeltec_ublox_driver.launch
使用UBLOX协议在RVIZ中记录经纬度轨迹(仅WHEELTEC G70)：
roslaunch wheeltec_gps_driver ublox_gps_path.launch

--gps topic: /ublox_gps_node/fix
--gps frame_id: navsat_link

---------------------------------------------------------------------------------------------

使用Unicore协议解析WHEELTEC G90模块(仅WHEELTEC G90)：
roslaunch wheeltec_gps_driver wheeltec_dual_rtk_driver_unicore.launch

使用Unicore协议在RVIZ中记录经纬度轨迹(仅WHEELTEC G90)：
roslaunch wheeltec_gps_driver unicore_gps_path_dualrtk.launch


--gps topic: /gps/fix
--gps frame_id: navsat_link
--utm pose topic:/gps/utm_pose
--heading topic:/gps/euler
