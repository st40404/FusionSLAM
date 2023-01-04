#!/usr/bin/env bash

cd ./editor

for i in *rc; do
    mv ./${i} /home/${USER}/.${i}
done
