#!/usr/bin/env bash

# Resolve absolute paths
DATASETS_DIR="$(realpath /home/donceykong/Datasets)"
SCRIPTS_DIR="$(realpath /home/donceykong/Desktop/ARPG/projects/CU-Multi/dataset_tools/ros2/conversion_scripts)"

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
  cu_multi_ros2

# Optional mounts and GPU support (uncomment as needed):
#   --gpus all \
#   --env="NVIDIA_VISIBLE_DEVICES=all" \
#   --env="NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility" \

docker exec -it cu_multi_ros2 /bin/bash

