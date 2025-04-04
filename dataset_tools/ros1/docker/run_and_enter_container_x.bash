#!/usr/bin/env bash

xhost +local:root

# Run Docker container with specified configurations
docker run -it -d --rm --privileged \
  --name cu_multi_container \
  --net=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  cu_multi_container

# Optional mounts and GPU support (uncomment as needed):
#   --gpus all \
#   --volume="/home/donceykong/Desktop/ARPG/projects/fall_2024/Lidar2OSM_FULL/Lidar2OSM/cu_multi/rosbag_tools:/root/data/rosbag_tools:rw" \
#   --volume="/media/donceykong/donceys_data_ssd/datasets/CU_MULTI/data:/root/data:rw" \
#   --volume="/media/donceykong/donceys_data_ssd/datasets/CU_MULTI/hunter_ws:/root/hunter_ws:rw" \
#   --env="NVIDIA_VISIBLE_DEVICES=all" \
#   --env="NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility" \

docker exec -it cu_multi_container /bin/bash

