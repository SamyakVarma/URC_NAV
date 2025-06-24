# URC_NAV Simulation Setup

A ROS2-based navigation stack for autonomous ground robots — this guide covers only the **simulation** setup, perfect for testing without physical hardware.

## System Requirements

- **Ubuntu 22.04**
- **ROS2 Humble** (recommended)
- **Gazebo Classic (Fortress or Garden)** or RViz for visualization

## 1. Install ROS2 Humble

Follow the official guide:  
 [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

After installation, set up your environment:

```bash
source /opt/ros/humble/setup.bash
```

Add this to your .bashrc:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## 4. Install Other dependencies
### Required ROS2 Packages

**Run in terminal**
```bash
sudo apt update
sudo apt install \
  ros-humble-rtab-map-ros \
  ros-humble-robot-localization \
  ros-humble-vision-opencv \
  ros-humble-image-transport \
  ros-humble-cv-bridge
```
### Required Python dependencies
```bash
pip install opencv-contrib-python numpy
```

## 3. Workspace Setup

Create and build the workspace:
```bash
mkdir -p ~/urc_sim_ws/src
cd ~/urc_sim_ws/src
git clone https://github.com/SamyakVarma/URC_NAV.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
You can also add this to .bashrc:
```bash
echo "source ~/urc_sim_ws/install/setup.bash" >> ~/.bashrc
```
## 4. Launch the Simulation

**Mars world with rover**
```bash
ros2 launch rover_gazebo mars.launch.py
```

**Perform visual SLAM on this**
```bash
ros2 launch rtab_map_ros rtabmap.launch.py use_sim_time:=true
```
**Perform hexagonal search pattern for ArUco Markers
```bash
ros2 run rover_controller rover_searcher
```
**Make Sure EKF is running properly with**
```bash
ros2 launch robot_localization ekf.launch.py
```
Sensors typically required:
- /imu/data

- /odom

- /gps/fix

The configs for the launch files can be found in **urc_nav/config**
