#!/bin/bash

RACER_DOCKER_CONTAINER_ID=$(sudo docker ps -aqf "name=ghost-racer")

if [ ! -z "$RACER_DOCKER_CONTAINER_ID" ];then
    # exec existing container
    echo "Opening existing ghost-racer's container [$RACER_DOCKER_CONTAINER_ID]"
    sudo docker exec -it $RACER_DOCKER_CONTAINER_ID /bin/bash
    exit 0
fi

ghost_racer_root_dir=$1
docker_ghost_racer_root_dir=/home/ghost/ghost-racer

if [ -z $1 ]; then
    ghost_racer_root_dir=$(pwd)
fi

echo "root directory of the ghost-racer: $ghost_racer_root_dir"

docker run -it --rm \
    -e "DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    --network host \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/:rw \
    -v $ghost_racer_root_dir:$docker_ghost_racer_root_dir \
    --name "ghost-racer" \
    jakubtomczak/ghost-racer