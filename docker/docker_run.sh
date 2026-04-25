#!/bin/bash

if [[ $# -lt 1 ]] ; then
  echo 'Usage: ./docker_run.sh <tag_name>   (same image tag you passed to docker_build.sh; any name you chose is fine)'
  exit 1
elif [[ $# -eq 1 ]] ; then
  tagname=$1

  SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
  PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

  xhost +local:docker
  docker run \
    --rm -ti \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ${PROJECT_ROOT}/drone_ws/src:/workspace/drone_ws/src \
    -v ${PROJECT_ROOT}/scripts:/workspace/scripts \
    -e QT_X11_NO_MITSHM=1 \
    --privileged \
    --network=host \
    --name quadcopter-sitl-env \
    --workdir /workspace \
    ${tagname} 
fi