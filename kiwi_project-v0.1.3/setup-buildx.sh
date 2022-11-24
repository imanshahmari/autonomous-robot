#!/bin/bash

sudo apt-get update
sudo apt-get upgrade
sudo apt-get install qemu-user-static
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
docker run --rm -t arm32v6/alpine uname -m
docker run --rm -t arm64v8/ubuntu uname -m
mkdir -p ~/.docker/cli-plugins
cd ~/.docker/cli-plugins/
wget https://github.com/docker/buildx/releases/download/v0.7.1/buildx-v0.7.1.linux-amd64
mv buildx-v0.7.1.linux-amd64 docker-buildx
chmod 755 docker-buildx
DOCKER_CLI_EXPERIMENTAL=enabled docker buildx create --name platformbuilder2 --use