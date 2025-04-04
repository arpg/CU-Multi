#!/usr/bin/env bash

xhost +local:root

# Run Docker container with specified configurations
docker run -it -d --rm --privileged \
  --name cu_multi_ros1 \
  --net=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  cu_multi_ros1

# Optional mounts and GPU support (uncomment as needed):
#   --gpus all \
#   --env="NVIDIA_VISIBLE_DEVICES=all" \
#   --env="NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility" \

docker exec -it cu_multi_ros1 /bin/bash

