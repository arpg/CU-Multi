#!/bin/bash

DOCKERFILE="./docker/Dockerfile_ros1"
IMAGE_TAG="cu_multi_ros1"
BUILD_CONTEXT="."

# Use the specified Dockerfile in the Docker build command
docker build -f $DOCKERFILE -t $IMAGE_TAG $BUILD_CONTEXT

