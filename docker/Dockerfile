# Build environment for the Gazebo adapter: Ubuntu 24.04 with Gazebo Jetty packages.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg lsb-release cmake g++ make \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && rm -rf /var/lib/apt/lists/*

# The gz-sensors package and its dependencies (installed in a separate layer to measure them).
# libgz-sensors10-dev does not pull in the gz-common profiler dev package that its CMake config requires,
# and its CMake config always looks for gz-rendering, which needs the OpenGL dev files.
ARG GZ_SENSORS_PACKAGE=libgz-sensors10-dev
RUN apt-get update && apt-get install -y --no-install-recommends ${GZ_SENSORS_PACKAGE} libgz-common7-profiler-dev libgl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
