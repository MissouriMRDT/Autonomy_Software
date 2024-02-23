#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
TORCH_VERSION="2.0.1"
TORCH_CUDA_VERSION="cu117"

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/pytorch/arm64/pytorch_${TORCH_VERSION}_arm64.deb"

# Check if the file exists
if curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${TORCH_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
else
    echo "Package version ${TORCH_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
    
    # Delete Old Packages
    rm -rf /tmp/pkg
    # rm -rf /tmp/libtorch

    # Create Package Directory
    mkdir -p /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/usr/local
    mkdir -p /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/DEBIAN

    # Create Control File
    echo "Package: pytorch-mrdt" > /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/DEBIAN/control
    echo "Version: ${TORCH_VERSION}" >> /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/DEBIAN/control
    echo "Maintainer: pytorch" >> /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/DEBIAN/control
    echo "Depends:" >> /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/DEBIAN/control
    echo "Architecture: arm64" >> /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/DEBIAN/control
    echo "Homepage: https://pytorch.org/cppdocs/" >> /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/DEBIAN/control
    echo "Description: A prebuilt version of Torch. Made by the Mars Rover Design Team." >> /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/DEBIAN/control

    # Download Torch
    git clone --depth 1 --branch v${TORCH_VERSION} --recurse-submodule https://github.com/pytorch/pytorch.git
    mkdir -p pytorch/build && cd pytorch/build

    # Install python dependencies for building libtorch.
    pip install -r ../requirements.txt
    
    # Build Torch
    cmake \
        -D CMAKE_INSTALL_PREFIX:PATH=/tmp/pkg/opencv_${TORCH_VERSION}_arm64/usr/local \
        -D BUILD_SHARED_LIBS:BOOL=OFF \
        -D CMAKE_BUILD_TYPE:STRING=Release \
        -D PYTHON_EXECUTABLE:PATH=`which python3` ..

    # Install Torch
    cmake --build . --target install

    # # Cleanup Install
    # rm -rf /tmp/pytorch

    # # Create Package
    # dpkg --build /tmp/pkg/pytorch_${TORCH_VERSION}_arm64

    # # Create Package Directory
    # mkdir -p /tmp/pkg/deb

    # # Copy Package
    # cp /tmp/pkg/pytorch_${TORCH_VERSION}_arm64.deb /tmp/pkg/deb/pytorch_${TORCH_VERSION}_arm64.deb
fi
