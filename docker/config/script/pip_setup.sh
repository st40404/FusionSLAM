#!usr/bin/env bash

python3 -m pip install --upgrade --force-reinstall pip
pip3 install -r ./pip/requirements.txt

# install python dependency for extract bag
pip2 install tqdm
pip2 install scipy