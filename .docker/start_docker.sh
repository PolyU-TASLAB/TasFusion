#!/bin/bash

# Define names for your image and container for easy reuse
IMAGE_NAME="toyslam-dev"
CONTAINER_NAME="toyslam_container"

# Define local output directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOCAL_OUTPUT_DIR="$PROJECT_ROOT/data/results"

# Create local output directory if it doesn't exist
mkdir -p "$LOCAL_OUTPUT_DIR"

# --- GUI Forwarding Setup ---
# Allow local connections to the X server
xhost +local:

# --- Docker Command ---
# Check if a container with the same name is already running
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Attaching to running container: $CONTAINER_NAME"
    docker exec -it $CONTAINER_NAME bash
# Check if a container with the same name exists but is stopped
elif [ "$(docker ps -aq -f status=exited -f name=$CONTAINER_NAME)" ]; then
    echo "Starting and attaching to existing container: $CONTAINER_NAME"
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME bash
# Otherwise, create and run a new container
else
    echo "Running a new container: $CONTAINER_NAME"
    echo "Mounting local output directory: $LOCAL_OUTPUT_DIR -> /data/results/"
    docker run -it \
        --name $CONTAINER_NAME \
        --rm \
        --net=host \
        --privileged \
        -e "DISPLAY=$DISPLAY" \
        -e "CSV_OUTPUT_DIR=/data/results/" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -v "$(pwd)":/root/catkin_ws \
        -v "$LOCAL_OUTPUT_DIR":/data/results/ \
        $IMAGE_NAME
fi

# --- Cleanup ---
# Disallow local connections to the X server after closing the container
xhost -local: