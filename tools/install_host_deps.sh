#!/usr/bin/env bash
# This script installs dependencies for docker, cross-compile support for containers, and the nvidia-container runtime.

echo "#######################
##  Add docker repo and install.
#######################"
# Install general packages for adding repos and keyrings.
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
# Add docker GPG key.
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
# Setup docker repo.
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
# Install docker.
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker.io docker-buildx-plugin docker-compose-plugin


echo "#######################
## Install Emulation Support.
#######################"
# Install packages for cross-compile support and emulation.
sudo apt-get install -y qemu binfmt-support qemu-user-static

echo "#######################
## Install Nvidia container runtime.
#######################"
# Add nvidia runtime GPG key.
curl -s -L https://nvidia.github.io/nvidia-container-runtime/gpgkey | \
  sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
# Add and setup repo.
curl -s -L https://nvidia.github.io/nvidia-container-runtime/$distribution/nvidia-container-runtime.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-runtime.list
sudo apt-get update
# Install container runtime.
sudo apt-get -y install nvidia-container-runtime
