#!/bin/bash

# VERSION_TAG=1.0
DOCKERFILE="./docker/Dockerfile_ROS2"
IMAGE_TAG="cu_multi_ROS2_container"

# Use the specified Dockerfile in the Docker build command
docker build -f $DOCKERFILE