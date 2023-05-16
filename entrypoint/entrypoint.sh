#!/bin/bash
set -e

source "/ros_entrypoint.sh"
source "/evolved5g/install/setup.bash"

echo "172.17.0.1      capifcore" >> /etc/hosts

jq -r .capif_host=\"$CAPIF_HOSTNAME\" /evolved5g/cfg/capif_registration.json >> /evolved5g/cfg/tmp.json && mv /evolved5g/cfg/tmp.json /evolved5g/cfg/capif_registration.json
jq -r .capif_http_port=\"$CAPIF_PORT_HTTP\" /evolved5g/cfg/capif_registration.json >> /evolved5g/cfg/tmp.json && mv /evolved5g/cfg/tmp.json /evolved5g/cfg/capif_registration.json
jq -r .capif_https_port=\"$CAPIF_PORT_HTTPS\" /evolved5g/cfg/capif_registration.json >> /evolved5g/cfg/tmp.json && mv /evolved5g/cfg/tmp.json /evolved5g/cfg/capif_registration.json
jq -r .capif_netapp_username=\"$CAPIF_USERNAME\" /evolved5g/cfg/capif_registration.json >> /evolved5g/cfg/tmp.json && mv /evolved5g/cfg/tmp.json /evolved5g/cfg/capif_registration.json

evolved5g register-and-onboard-to-capif --config_file_full_path="/evolved5g/cfg/capif_registration.json"

exec ros2 run localization_netapp cellid_node

exec "$@"
