# Documentation

clone the workspace,

cd URC_NAV

install the submodules:
rosdep install --from-paths src -r -y

Simulation world on mars with rtab: ros2 launch rover_gazebo mars.launch.py

Search pattern: ros2 run rover_controller rover_searcher

aruco marker detection: ros2 run aruco_marker_detection aruco_detection_node

Requirements:
rtab_map
robot_localization
opencv_contrib_python

