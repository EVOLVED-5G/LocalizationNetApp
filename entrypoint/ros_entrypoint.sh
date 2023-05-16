#!/bin/bash
set -e

source "/opt/ros/foxy/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///evolved5g/cfg/$NETWORK.xml

