#!/usr/bin/env bash

################################# PARAMETERS #################################
# Get script path
file_dir=$(dirname $0)

# Get parameter from system
user=$(id -un)
group=$(id -gn)
uid=$(id -u)
gid=$(id -g)

# Get WorkSpace parameters
workspace=$(readlink -f "$0")
workspace=${workspace%_ws*}
workspace_name=${workspace##*/}

# Docker image name
image_name=${workspace_name}

# Enable docker build kit function
export DOCKER_BUILDKIT=1

#################################### MAIN ####################################
# Build docker images
docker build -t ${user}/${image_name} \
    --build-arg USER=${user} \
    --build-arg UID=${uid} \
    --build-arg GROUP=${group} \
    --build-arg GID=${gid} \
    -f ${file_dir}/Dockerfile ${file_dir}

#     --progress=plain \
