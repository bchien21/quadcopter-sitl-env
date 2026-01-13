#!/bin/bash

# Exit immediately if any command fails
set -e

if [[ $# -lt 2 ]] ; then
  echo 'Usage: ./docker_build.sh <dockerfile_path> <tag_name>'
  exit 1
fi

dockerfile=$1
tagname=$2

echo "Building ${tagname} from ${dockerfile}..."

# DOCKER_BUILDKIT=1 enables the modern, faster build engine
DOCKER_BUILDKIT=1 docker build \
  --rm \
  --build-arg UID=$(id -u) \
  --build-arg GID=$(id -g) \
  --build-arg UNAME=$(whoami) \
  -f "${dockerfile}" \
  -t "${tagname}" .

# This line will only run if the build command succeeded (because of set -e)
echo "----------------------------------------------------"
echo "✅ Successfully built Docker image with tag: ${tagname}"