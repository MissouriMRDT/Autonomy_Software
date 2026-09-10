#!/bin/bash
# -----------------------------------------------------------------------------
# Script Name: run_autonomy_docker.sh
# Description:
#   This script launches the MRDT Autonomy Jetpack Docker container with
#   NVIDIA GPU support and host-level access for development or runtime testing.
#
#   It mounts key directories and devices required for camera input, calibration
#   files, and model resources (e.g., ZED camera data), as well as shares X11 and
#   WSLg resources for GUI visualization. The container starts an interactive
#   Fish shell environment.
#
# Key Options:
#   --runtime=nvidia   Enables GPU acceleration for ZED and deep learning models.
#   --network=host     Shares the host network stack for direct hardware access.
#   --privileged       Grants full access to host devices (required for USB and camera).
#   -v ...:delegated   Mounts host directories into container for data access.
#   --rm               Automatically removes the container when exited.
#
# Usage:
#   ./run_autonomy_docker.sh
#
# Image Source:
#   ghcr.io/missourimrdt/autonomy-jammy:2026-05-01-15-42-01
#
# Author: MRDT Autonomy Team
# -----------------------------------------------------------------------------

docker run -it --rm --runtime=nvidia --network=host --privileged \
-v ../data/models/zed:/usr/local/zed/resources:delegated \
-v ../data/calibrations/zed:/usr/local/zed/settings:delegated \
-v /etc/localtime:/etc/localtime:ro \
-v /dev/bus/usb:/dev/bus/usb \
-v /mnt/wslg:/mnt/wslg \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v ../../Autonomy_Software:/opt/Autonomy_Software \
ghcr.io/missourimrdt/autonomy-jammy:2026-05-01-15-42-01 /usr/bin/fish
