# ROS2-GO2

This repository contains a ROS2 workspace with rviz and gazebo integrated with Go2.
Rviz is used for visualization and is useful for testing the robot's perception and navigation capabilities, while Gazebo provides a simulation environment for testing and development in real world scenarios. For now, the world used is the empty world provided by Gazebo, but it can be modified to include more complex environments.

## Installation and Usage
To set up and run the ROS2-Go2 workspace, follow these steps:
1. Clone the repository to your local machine:
```bash
git clone https://github.com/palingenesys/Ros2-go2/
cd Ros2-go2
```
2. Build the Docker image:
```bash
cd go2_ws/docker
./build.sh
```

3. Start the Docker container:
```bash
./start.sh
```

4. Install tmux inside the Docker container:
```bash
apt-get update
apt-get install -y tmux
```

5. Inside the Docker container, source the ROS2 and workspace setup files:
```bash
tmux new -s s1
cd ~/go2_ws/src
colcon build
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash
```

6. Launch the Gazebo simulation with the Go2 robot:
```bash
ros2 launch go2_bringup go2_gazebo.launch.py
```

7. In a new tmux session inside the Docker container, launch RViz:
```bash
tmux new -s s2
colcon build
source ~/go2_ws/install/setup.bash
ros2 launch go2_bringup go2_rviz.launch.py
```
