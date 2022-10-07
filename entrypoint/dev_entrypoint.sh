#!/bin/bash
set -e

source "/opt/ros/foxy/setup.bash"
source "/evolved5g/install/setup.bash"

export NEF_HOST="http://nef.apps.ocp-epg.hi.inet/"
#export NEF_HOST="http://localhost:8888"

exec ros2 run localization_netapp cellid_node
