#!/usr/bin/env bash

################################# FUNCTIONS #################################
################################
# Setup various shell configurations
# Arguments:
# - $1 (String) - Shell type
################################
ShellConfig_fn() {
    # Set all to lowercase
    SHELL=$(echo ${1} | tr [A-Z] [a-z])

    # Setup zsh configurations
    if [ ${SHELL} == "zsh" ]; then
        # zsh theme - Powerlevel10k
        echo "source ~/.powerlevel10k/powerlevel10k.zsh-theme" >> \
            /home/${USER}/.${SHELL}rc
        echo "source ~/.p10k.zsh" >>/home/${USER}/.${SHELL}rc
        mv ./shell/zsh/powerlevel10k /home/${USER}/.powerlevel10k
        mv ./shell/zsh/.p10k.zsh /home/${USER}/

        echo "alias ll='ls -alF'" >>/home/${USER}/.${SHELL}rc
        echo "alias la='ls -A'" >>/home/${USER}/.${SHELL}rc
        printf "alias l='ls -CF'\n" >>/home/${USER}/.${SHELL}rc

    # Setup bash configurations
    elif [ ${SHELL} == "bash" ]; then
        # display git branch
        cat ./shell/bash/git_config.sh >>/home/${USER}/.${SHELL}rc
    fi

    # Setup the configuration of the generic shell
    echo "alias eb='vim ~/.${SHELL}rc'" >>/home/${USER}/.${SHELL}rc
    echo "alias sb='source ~/.${SHELL}rc'" >>/home/${USER}/.${SHELL}rc
    echo "alias wb='source ./devel/setup.${SHELL}'" >>/home/${USER}/.${SHELL}rc
    chown ${USER}:${GROUP} /home/${USER}/.${SHELL}rc

}

################################
# Setup byobu configurations
# Arguments:
# - $1 (String) - Shell type
################################
BybouConfig_fn() {
    echo "set -g default-shell /bin/${1}" >/home/${USER}/.byobu/.tmux.conf
    echo "set -g default-command /bin/${1}" >>/home/${USER}/.byobu/.tmux.conf

}
#################################### MAIN ####################################
# bash
ShellConfig_fn "bash"

# zsh
ShellConfig_fn "zsh"

# byobu
mkdir -p /home/${USER}/.byobu
BybouConfig_fn ${BYOBU_SHELL}
chown -R ${USER}:${GROUP} /home/${USER}/.byobu

# terminator
mkdir -p /home/${USER}/.config/
cp -r ./shell/terminator /home/${USER}/.config/
chown -R ${USER}:${GROUP} /home/${USER}/.config
