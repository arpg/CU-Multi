#!/usr/bin/env bash

# Give privledge for screen sharing
xhost +local:root

# Resolve absolute paths to:
# 1. Dir containing CU-Multi Dataset
DATASETS_DIR="$(realpath <ABS-PATH-TO>/Datasets)"
# 2. Path to ROS1 conversion scripts dir
SCRIPTS_DIR="$(realpath <ABS-PATH-TO>/CU-Multi/dataset_tools/ros1/conversion_scripts)"
# 3. Path to ROS1 workspace for testing
ROS2_WS_DIR="$(realpath <ABS-PATH-TO>/<your_ros2_ws>)"

# Run Docker container with specified configurations
docker run -it -d --rm --privileged \
  --name cu_multi_ros1 \
  --net=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$DATASETS_DIR:/root/Datasets:rw" \
  --volume="$SCRIPTS_DIR:/root/conversion_scripts:rw" \
  cu_multi_ros1

# Optional mounts and GPU support (uncomment as needed):
#   --gpus all \
#   --env="NVIDIA_VISIBLE_DEVICES=all" \
#   --env="NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility" \

docker exec -it cu_multi_ros1 /bin/bash

