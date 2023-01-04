#!/usr/bin/env bash

################################# PARAMETERS #################################
# Get script path
file_dir="$(dirname $0)"

# Get WorkSpace parameters
workspace=$(readlink -f "${0}")
workspace=${workspace%_ws*}
workspace_name=${workspace##*/}
workspace_path=${workspace}"_ws"

# Docker image and container name
image_name=${workspace_name}
container_name=${workspace_name}

# Start sharing xhost
# You can also comment it, if you report an error
# xhost +local:root

################################# FUNCTIONS #################################
################################
# Get the GraphicsCard label
# Arguments:
# - $1 (Boolean) - Debug mode
# Returns:
# --gpus all is NVIDIA GraphicsCard
# "" is not supported
################################
function get_graph_card() {
    graphics_card=$(lspci | grep VGA)

    if ${1}; then
        echo $graphics_card
    fi

    graphics_card=${graphics_card##*:}
    graphics_card=${graphics_card#* }
    graphics_card=${graphics_card%% *}
    graphics_card=$(echo $graphics_card | tr [a-z] [A-Z])

    if [ $graphics_card == "NVIDIA" ]; then
        echo "--gpus all"
    else
        echo ""
    fi
}

################################
# run docker container
# Arguments:
# - $1 (String) - GPU paramters
################################
docker_run() {
    docker run --rm \
        --net=host \
        --ipc=host \
        $1 \
        --privileged \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v ${HOME}/.Xauthority:$docker/.Xauthority \
        -v ${workspace_path}:${HOME}/work \
        -v /dev:/dev \
        -v /etc/timezone:/etc/timezone:ro \
        -v /etc/localtime:/etc/localtime:ro \
        -e XAUTHORITY=${HOME}_folder/.Xauthority \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -it --name $container_name $(id -un)/${image_name}
}

#################################### MAIN ####################################
gpu_flag="$(get_graph_card false)"
docker_run "${gpu_flag}"
