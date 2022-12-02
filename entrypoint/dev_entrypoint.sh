#!/bin/bash
set -e

source "/opt/ros/foxy/setup.bash"
source "/evolved5g/install/setup.bash"

echo "172.17.0.1      capifcore" >> /etc/hosts

sed "s/capifcore/$CAPIF_HOST/" /evolved5g/cfg/capif_registration.json
sed "s/8080/$CAPIF_HTTP_PORT/" /evolved5g/cfg/capif_registration.json
sed "s/443/$CAPIF_HTTPS_PORT/" /evolved5g/cfg/capif_registration.json


evolved5g register-and-onboard-to-capif --config_file_full_path="/evolved5g/cfg/capif_registration.json"

exec ros2 run localization_netapp cellid_node

exec "$@"
