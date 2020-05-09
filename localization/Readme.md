## For Localization<br />
roscore<br />
roslaunch localization localization_vanilla.launch<br />
rviz rviz<br />
rqt<br />
<br />
## First try for Path planning<br />
<br />
roscore<br />
roslaunch localization path_planning.launch<br />
rviz rviz<br />
<br />
rostopic echo /final_goal # Not necessarily needed<br />
rosrun localization path_planning.py<br />
<br />
## Path Planning: If nothing works then this is the sequence<br />
<br />
roscore<br />
roslaunch dd2419_simulation simulation.launch<br />
roslaunch dd2419_simulation aruco.launch gui:=false<br />
<br />
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map cf1/odom<br />
rosrun tf2_ros static_transform_publisher 0.01 0 0.02 -1.54 0 -1.54 cf1/base_link cf1/camera_link<br />
rosrun part2 displaymapmarkers ~/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/tutorial_1.world.json<br />
<br />
rosrun map_server map_server /home/akanshu/dd2419_ws/src/localization/2Dmap.yaml<br />
rosrun localization Readfinalgoal.py<br />
<br />
rviz<br />
rostopic echo /final_goal<br />
rosrun localization path_planning.py<br />

