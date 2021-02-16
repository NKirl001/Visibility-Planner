# Visibility-Planner
Naadir Kirlew Master Thesis 2021

-Proposes Visibility Planner for exploration of unknown environments

-built in Ubuntu 16.04 using ROS Kinetic

--------------------------------------------------------
Directory is divided into 3 sections 
1) 2wd contains all files pertaining to a differntial drive robot 
2) 4wd contains all files pertaining to a 4 wheeled skid steer robot
3) multi_robot_collab contains all file for multi robot simulation

---------------------------------------------------------
To Launch a single robot differntial drive simulation:
-requires 3 terminals
1) open a terminal
2) navigate to this directory 

In terminal 1

3) Enter: $ roslaunch two_wd_worlds world.launch

In terminal 2

4) Launch Gmapping: $roslaunch two_wd_scripts gmapping.launch 

In terminal 3

5) Enter: $ rosrun two_wd_scripts robot_control_test.py 

-------------------------------------------------------------
To Launch a Multi-robot simulation:
-requires 9 terminals
1) open a terminal
2) navigate to this directory 
In terminal 1 run 
3) $ roslaunch work_tog multi_bot.launch 

In terminal 2 run

4) $ ROS_NAMESPACE=bot_0 roslaunch work_tog gmapping_multi.launch base_frame:=bot_0/link_chassis odom_frame:=bot_0/odom scan_topic:=/bot_0/laser/scan set_map_frame:=bot_0/map

In terminal 3 run 

5) $ ROS_NAMESPACE=bot_1 roslaunch work_tog gmapping_multi.launch base_frame:=bot_1/link_chassis odom_frame:=bot_1/odom scan_topic:=/bot_1/laser/scan set_map_frame:=bot_1/map

In terminal 4 run 

6) $ ROS_NAMESPACE=bot_2 roslaunch work_tog gmapping_multi.launch base_frame:=bot_2/link_chassis odom_frame:=bot_2/odom scan_topic:=/bot_2/laser/scan set_map_frame:=bot_2/map

In terminal 5 run 

7) $ roslaunch work_tog multi_map_merge.launch 

In terminal 6 run 

8) $ rosrun rviz rivz

the remaining terminals are used to send commands to each robot. 
