#!/bin/bash
set -e

source "/opt/ros/foxy/setup.bash"
source "/evolved5g/install/setup.bash"

echo "172.17.0.1      capifcore" >> /etc/hosts

evolved5g register-and-onboard-to-capif --config_file_full_path="/evolved5g/cfg/capif_registration.json"

exec ros2 run localization_netapp cellid_node

exec "$@"
