#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
TORCH_VERSION="2.2.2"
TORCH_CUDA_VERSION="cu121"
FORCE_BUILD=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --force|-f)
            FORCE_BUILD=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/pytorch/arm64/pytorch_${TORCH_VERSION}_arm64.deb"

# Check if the file exists
if [[ "$FORCE_BUILD" == false ]] && curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${TORCH_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
else
    echo "Package version ${TORCH_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT

    # Install dependencies.
    apt update && apt install -y python3-numpy python-is-python3
    
    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/libtorch

    # Create Package Directory
    mkdir -p /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/usr/local
    mkdir -p /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/DEBIAN

    # Create Control File
    {
        echo "Package: pytorch-mrdt"
        echo "Version: ${TORCH_VERSION}"
        echo "Maintainer: pytorch"
        echo "Depends:"
        echo "Architecture: arm64"
        echo "Homepage: https://pytorch.org/cppdocs/"
        echo "Description: A prebuilt version of Torch. Made by the Mars Rover Design Team."
    } > /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/DEBIAN/control

    # Download Torch
    git clone --depth 1 --branch v${TORCH_VERSION} --recurse-submodule https://github.com/pytorch/pytorch.git
    # Install python dependencies for building libtorch.
    cd pytorch && pip3 install -r requirements.txt
    
    # mkdir pytorch-build
    cd /tmp/pytorch-build    
    # Build Torch
    cmake -DBUILD_SHARED_LIBS:BOOL=ON -DCMAKE_BUILD_TYPE:STRING=Release -DPYTHON_EXECUTABLE:PATH=`which python3` -DCMAKE_INSTALL_PREFIX:PATH=../pytorch-install ../pytorch

    # Install Torch
    cmake --build . --target install

    # Check if CMake was successful with exit code 0.
    if [ $? -eq 0 ]; then
        # Copy Torch
        mkdir -p /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/usr/bin
        mkdir -p /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/usr/include
        mkdir -p /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/usr/lib
        mkdir -p /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/usr/share
        cp -r /tmp/pytorch-install/bin/* /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/usr/bin/
        cp -r /tmp/pytorch-install/include/* /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/usr/include/
        cp -r /tmp/pytorch-install/lib/* /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/usr/lib/
        cp -r /tmp/pytorch-install/share/* /tmp/pkg/pytorch_${TORCH_VERSION}_arm64/usr/share/

        # Cleanup Install
        rm -rf /tmp/pytorch
        rm -rf /tmp/pytorch-build
        rm -rf /tmp/pytorch-install

        # Create Package
        dpkg --build /tmp/pkg/pytorch_${TORCH_VERSION}_arm64

        # Create Package Directory
        mkdir -p /tmp/pkg/deb

        # Copy Package
        cp /tmp/pkg/pytorch_${TORCH_VERSION}_arm64.deb /tmp/pkg/deb/pytorch_${TORCH_VERSION}_arm64.deb
    else
        # Cleanup Install
        rm -rf /tmp/pytorch

        # Return non success exit code.
        exit 1
    fi
fi
