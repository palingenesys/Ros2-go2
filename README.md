# ROS2-GO2

This repository contains a ROS2 workspace with rviz and gazebo integrated with Go2.
Rviz is used for visualization and is useful for testing the robot's perception and navigation capabilities, while Gazebo provides a simulation environment for testing and development in real world scenarios. For now, the world used is the empty world provided by Gazebo, but it can be modified to include more complex environments.

## Preliminary setup for users with NVIDIA GPU
If you have an NVIDIA GPU, Gazebo can take advantage of it inside the container to achieve better performance.
1. Make sure you have installed the NVIDIA driver on your host machine. You can find the installation instructions [here](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/#driver-installation).
2. Install the NVIDIA Container Toolkit on your host machine. You can find the installation instructions [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html). For Ubuntu, run what follows:
```bash
sudo apt-get update && sudo apt-get install -y --no-install-recommends curl gnupg2

curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
&& curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \

sudo apt-get update

export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.18.0-1
sudo apt-get install -y \
      nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
```
3. Configure docker to use the NVIDIA Container Toolkit:
```bash
sudo nvidia-ctk runtime configure --runtime=docker
```
4. Restart the docker daemon:
```bash
sudo systemctl restart docker
```

## Installation and Usage
To set up and run the ROS2-Go2 workspace, follow these steps:

1. Clone the repository to your local machine, in the $HOME directory (otherwise modify ./go2_ws/docker/start.sh accordingly):
```bash
cd
git clone --recursive https://github.com/palingenesys/Ros2-go2/
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
In case you have an NVIDIA GPU, run the following to enable GPU acceleration:
```bash
./start.sh --gpu
```

4. Create multiple terminals to run different commands:
```bash
# run these commands for opening new terminal while running the container
tmux new -s s1
tmux new -s s2

# open an active session (where s1 and s2 are the sessions names)
tmux a -t s1
```

5. Inside the first session, source the ROS2 and workspace setup files:
```bash
cd /workspace/src
colcon build
source /opt/ros/humble/setup.bash
source /workspace/src/install/setup.bash
```

6. Launch the Gazebo simulation with the Go2 robot:
```bash
ros2 launch go2_config gazebo.launch.py
```

7. Or if you want to launch also Rviz, in a new tmux session inside the Docker container, launch the following node:
```bash
cd /workspace/src
colcon build
source /workspace/src/install/setup.bash
ros2 launch go2_config gazebo.launch.py rviz:=true
```

8. In order to manually control the robot, in a new tmux session launch the following:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```