#!/bin/bash
set -e

WS_DIR="$HOME/Ros2-go2/go2_ws"
IMAGE_NAME="go2_humble"

# X11 permissions
xhost +local:docker > /dev/null

# If --gpu option, run the container with support for nvidia acceleration
if [ "$1" == "--gpu" ]; then
    echo "Running with GPU support"
    
    docker run -it --rm \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$WS_DIR":/workspace \
        -w /workspace \
        --network host \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
        --gpus all \
        $IMAGE_NAME \
        bash -c "source /opt/ros/humble/setup.bash && \
             [ -f install/setup.bash ] && source install/setup.bash; \
             bash"
else
    echo "Running without GPU support"

    docker run -it --rm \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$WS_DIR":/workspace \
        -w /workspace \
        --network host \
        $IMAGE_NAME \
        bash -c "source /opt/ros/humble/setup.bash && \
                [ -f install/setup.bash ] && source install/setup.bash; \
                bash"
fi