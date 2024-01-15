#!/usr/bin/env bash

echo -e "Building iaac_ur_commander:lastest image"

DOCKER_BUILDKIT=1 \
docker build --pull --rm -f ./.docker/Dockerfile \
--build-arg BUILDKIT_INLINE_CACHE=1 \
--target dev \
--tag iaac_ur_commander:latest .