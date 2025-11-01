#!/bin/bash
set -e

WS_DIR="$HOME/Ros2-go2/go2_ws"
IMAGE_NAME="go2_humble"

# X11 permissions
xhost +local:docker > /dev/null

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
