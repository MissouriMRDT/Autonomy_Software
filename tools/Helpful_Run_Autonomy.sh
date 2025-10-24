#!/bin/bash
docker run -it --rm --runtime=nvidia --network=host --privileged \
-v ./Autonomy_Software/data/models/zed:/usr/local/zed/resources:delegated \
-v ./Autonomy_Software/data/calibrations/zed:/usr/local/zed/settings:delegated \
-v /etc/localtime:/etc/localtime:ro \
-v /dev/bus/usb:/dev/bus/usb \
-v /mnt/wslg:/mnt/wslg \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v ./Autonomy_Software:/opt/Autonomy_Software \
ghcr.io/missourimrdt/autonomy-jetpack:2025-04-19-04-17-28 /usr/bin/fish
