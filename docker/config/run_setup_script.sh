#!/usr/bin/env bash

script=(./script/*_setup.sh)

for current in ${script[@]}
do
    bash ${current}
done
