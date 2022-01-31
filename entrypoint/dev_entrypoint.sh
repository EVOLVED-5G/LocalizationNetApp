#!/bin/bash
set -e

source "/opt/ros/foxy/setup.bash"
source "/evolved5g/install/setup.bash"

exec ros2 run localization_netapp cellid_node
