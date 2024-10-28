#! /usr/bin/bash

xhost +local:docker

docker run \
-it --rm \
--privileged \
--gpus all \
--name marsim_cont \
--env="XDG_RUNTIME_DIR=/tmp/runtime-root" \
--env="DISPLAY=$DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="MESA_GL_VERSION_OVERRIDE=3.3" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix" \
--volume="/home/yhxing/Docker_WS/marsim1.2:/home/marsim_ws:rw" \
--runtime=nvidia \
marsim1.2_dock 
