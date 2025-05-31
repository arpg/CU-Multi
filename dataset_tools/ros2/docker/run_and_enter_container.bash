#!/usr/bin/env bash

# Resolve absolute paths to:

# 1. Dir containing CU-Multi Dataset
DATASETS_DIR="$(realpath <ABS-PATH-TO>/Datasets)"
# 2. CU-Multi/dataset_tools/ros2/conversion_scripts
SCRIPTS_DIR="$(realpath /<ABS-PATH-TO>/CU-Multi/dataset_tools/ros2/conversion_scripts)"
# 3. ROS2 WS, in case you would like to test methods on the CU-Multi Dataset ;)
ROS2_WS_DIR="$(realpath <ABS-PATH-TO>/<your_ros2_ws>)"

# Give privledge for screen sharing
xhost +local:root

# Run Docker container
docker run -it -d --rm --privileged \
  --name cu_multi_ros2 \
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$DATASETS_DIR:/root/Datasets:rw" \
  --volume="$SCRIPTS_DIR:/root/conversion_scripts:rw" \
  --volume="$ROS2_WS_DIR:/root/<your_ros2_ws>:rw" \
  cu_multi_ros2

# Optional mounts and GPU support (uncomment as needed):
#   --gpus all \
#   --env="NVIDIA_VISIBLE_DEVICES=all" \
#   --env="NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility" \

docker exec -it cu_multi_ros2 /bin/bash

