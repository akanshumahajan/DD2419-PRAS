How to run the code

0. Install the map aruco.world.json which contains a the ArUco marker 2. If another map is used, remember to change in MapPlot.py.
    cp ./aruco.world.json ~/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/
    # For the simulation; Rosrun the map and edit the line in simulation.launch with the map name
    rosrun dd2419_simulation json_to_world.py aruco.world
    nano ~/dd2419_ws/src/course_packages/dd2419_simulation/launch/simulation.launch

1. Either launch the simulation or the actual drone
    # Gazebo
    roslaunch dd2419_simulation simulation.launch
    roslaunch dd2419_simulation aruco.launch gui:=false
    # The drone
    roslaunch dd2419_launch base.launch ch:=XX

2. Create a constant transform between base_link and camera_link
    rosrun tf2_ros static_transform_publisher 0.01 0 0.02 -1.5707963267948966 0.0 -1.5707963267948966 cf1/base_link cf1/camera_link

3. Run the three background nodes
    rosrun PACKAGE MapPlot.py
    rosrun PACKAGE TransformTranslation.py
    rosrun PACKAGE ArUcoDetect.py

4. Start RViz
    rviz

5. "Fly you fools!"
    rosrun PACKAGE FlyControl.py
