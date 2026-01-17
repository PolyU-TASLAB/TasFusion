#!/bin/bash
set -e

# Source the main ROS setup file
source "/opt/ros/noetic/setup.bash"

# Automatically compile the workspace every time the container starts
echo "--- Running catkin_make to compile workspace ---"
catkin_make 
echo "--- Compilation finished ---"

# Source the local workspace for this initial shell
source "/root/catkin_ws/devel/setup.bash"
echo "Workspace sourced for this terminal."

# Execute the command passed into the container (usually 'bash')
exec "$@"