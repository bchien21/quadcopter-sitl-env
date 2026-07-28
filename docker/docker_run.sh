#!/bin/bash

if [[ $# -gt 1 ]] ; then
  echo 'Usage: ./docker_run.sh [tag_name]'
  exit 1
else
  tagname="${1:-drone_slam}"

  SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
  PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

  xhost +local:docker
  docker run \
    --rm -ti \
    --gpus all \
    -e DISPLAY="$DISPLAY" \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${PROJECT_ROOT}/drone_ws/src:/workspace/drone_ws/src" \
    -v "${PROJECT_ROOT}/scripts:/workspace/scripts" \
    -e QT_X11_NO_MITSHM=1 \
    --privileged \
    --network=host \
    --name quadcopter-sitl-env \
    --workdir /workspace \
    "${tagname}"
fi
