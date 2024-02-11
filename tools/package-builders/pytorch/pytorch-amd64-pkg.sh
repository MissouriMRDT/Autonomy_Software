#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
TORCH_VERSION="2.0.1"
TORCH_CUDA_VERSION="cu117"

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/torch/amd64/torch_${TORCH_VERSION}_amd64.deb"

# Check if the file exists
if curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${TORCH_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
else
    echo "Package version ${TORCH_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
    
    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/libtorch

    # Create Package Directory
    mkdir -p /tmp/pkg/torch_${TORCH_VERSION}_amd64/usr/local
    mkdir -p /tmp/pkg/torch_${TORCH_VERSION}_amd64/DEBIAN

    # Create Control File
    echo "Package: torch-mrdt" > /tmp/pkg/torch_${TORCH_VERSION}_amd64/DEBIAN/control
    echo "Version: ${TORCH_VERSION}" >> /tmp/pkg/torch_${TORCH_VERSION}_amd64/DEBIAN/control
    echo "Maintainer: torch" >> /tmp/pkg/torch_${TORCH_VERSION}_amd64/DEBIAN/control
    echo "Depends:" >> /tmp/pkg/torch_${TORCH_VERSION}_amd64/DEBIAN/control
    echo "Architecture: amd64" >> /tmp/pkg/torch_${TORCH_VERSION}_amd64/DEBIAN/control
    echo "Homepage: https://pytorch.org/cppdocs/" >> /tmp/pkg/torch_${TORCH_VERSION}_amd64/DEBIAN/control
    echo "Description: A prebuilt version of Torch. Made by the Mars Rover Design Team." >> /tmp/pkg/torch_${TORCH_VERSION}_amd64/DEBIAN/control

    # Download Torch
    wget -O torch.zip https://download.pytorch.org/libtorch/${TORCH_CUDA_VERSION}/libtorch-cxx11-abi-shared-with-deps-${TORCH_VERSION}%2B${TORCH_CUDA_VERSION}.zip
    unzip torch.zip
    # rm torch.zip
    cd libtorch

    # Install Torch
    mkdir -p /tmp/pkg/torch_${TORCH_VERSION}_amd64/usr/include
    mkdir -p /tmp/pkg/torch_${TORCH_VERSION}_amd64/usr/lib
    mkdir -p /tmp/pkg/torch_${TORCH_VERSION}_amd64/usr/share
    cp -r /tmp/libtorch/include/* /tmp/pkg/torch_${TORCH_VERSION}_amd64/usr/include/
    cp -r /tmp/libtorch/lib/* /tmp/pkg/torch_${TORCH_VERSION}_amd64/usr/lib/
    cp -r /tmp/libtorch/share/* /tmp/pkg/torch_${TORCH_VERSION}_amd64/usr/share/


    # Cleanup Install
    rm -rf libtorch

    # Create Package
    dpkg --build /tmp/pkg/torch_${TORCH_VERSION}_amd64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/torch_${TORCH_VERSION}_amd64.deb /tmp/pkg/deb/torch_${TORCH_VERSION}_amd64.deb
fi
