#!/bin/bash

nvidia-docker run --rm -it --gpus all \
 --env="DISPLAY=$DISPLAY" \
 --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
 --volume="$HOME/.ssh:/home/user/.ssh:ro" \
 --name tom_centauro \
 --network host \
 --privileged \
 arturolaurenzi/tom_centauro:latest \
 x-terminal-emulator
