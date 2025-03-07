#!/bin/bash

# VERSION_TAG=1.0
DOCKERFILE="./docker/Dockerfile"
IMAGE_TAG="cu_multi_container"

# clone repositories
# bash clone_repos.sh

# Use the specified Dockerfile in the Docker build command
docker build -f $DOCKERFILE
