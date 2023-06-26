########## FROZEN STAGE ##########
FROM ros:foxy AS frozen_stage

########## CACHE BREAKER STAGE ##########
FROM  frozen_stage AS cache_breaker_stage

# install CI dependencies
RUN apt-get update && apt-get install -q -y \
  ccache \
  lcov \
  && rosdep update \
  && rm -rf /var/lib/apt/lists/*

COPY src/cfg /evolved5g/cfg
COPY src/localization_netapp /evolved5g/src/localization_netapp
COPY src/evolvedApi /evolved5g/src/evolvedApi

RUN . /opt/ros/foxy/setup.sh \
  &&  rosdep keys --from-paths /evolved5g/src --ignore-src --rosdistro foxy | \
  xargs rosdep resolve --rosdistro foxy | \
  awk '/#apt/{getline; print}' > /rosdep_requirements.txt

WORKDIR /evolved5g/cfg
RUN mkdir capif_onboarding

########## BASE STAGE ##########
FROM frozen_stage AS base_stage

COPY --from=cache_breaker_stage /rosdep_requirements.txt /rosdep_requirements.txt

# Install Husarnet Client and deppendencies
RUN apt-get install ca-certificates \
  && apt update -y && apt install -y curl gnupg2 systemd 

RUN apt update \
  && apt install -y --no-install-recommends --no-upgrade $(cat /rosdep_requirements.txt) ros-foxy-rmw-cyclonedds-cpp

RUN apt-get install libcurl4-openssl-dev -y \
  && apt-get install libcurlpp-dev -y \
  && apt-get install nlohmann-json3-dev 

########## DEV BASE STAGE ##########
FROM base_stage AS dev_base_stage

RUN apt update \
  && apt install -y --no-install-recommends \
  python3-colcon-common-extensions \
  ros-foxy-diagnostic-updater \
  ros-foxy-tf2 \
  libboost-dev \
  python3-matplotlib \
  python3-numpy \
  python3-dev \
  python3-tk \
  python3-pip \
  libyaml-cpp-dev \
  jq

COPY pip_dependencies.txt /pip_dependencies.txt

RUN pip3 install -r /pip_dependencies.txt

COPY --from=cache_breaker_stage /evolved5g /evolved5g

########## BUILD STAGE ##########
FROM dev_base_stage AS build_stage

WORKDIR /evolved5g
RUN . /opt/ros/foxy/setup.sh \
  && colcon build


########## DEV STAGE ##########
FROM build_stage AS dev_stage

COPY entrypoint/entrypoint.sh /entrypoint.sh
COPY entrypoint/ros_entrypoint.sh /ros_entrypoint.sh

# Local configuration, to be set according to the NEF location
ENV NEF_ADDRESS="host.docker.internal"
ENV NEF_PORT="4443"
ENV NEF_USER="admin@my-email.com"
ENV NEF_PASSWORD="pass"
ENV VAPP_ADDRESS=
ENV PATH_TO_CERTS="/evolved5g/cfg/capif_onboarding"
ENV CAPIF_HOSTNAME="capifcore"
ENV CAPIF_PORT_HTTP="8080"
ENV CAPIF_PORT_HTTPS="443"
ENV CALLBACK_ADDRESS="localization:8000"
ENV UE_EXTERNAL_ID_1="10003@domain.com"
ENV UE_EXTERNAL_ID_2="10002@domain.com"
ENV CAPIF_USERNAME="user300"
ENV NETWORK="local"
ENV ROS_DOMAIN_ID="1"
ENV ENVIRONMENT="development"

# DDS Variables
ENV DDS_IFACE="auto"
ENV DDS_EXTERNAL_ADDRESS="10.11.23.49"
ENV DDS_PARTICIPANT_INDEX="auto"
ENV DDS_MAX_PARTICIPANT_INDEX="3"

ENTRYPOINT ["/entrypoint.sh"]
CMD tail -f /dev/null

########## PRODUCTION BUILD STAGE ##########
FROM dev_base_stage AS prod_build_stage


########## PRODUCTION STAGE ##########
FROM base_stage AS prod_stage


########## CONTINUOUS INTEGRATION STAGE ##########
FROM dev_stage AS ci_stage


########## LIVE TEST STAGE ##########
FROM dev_stage AS live_test_stage

