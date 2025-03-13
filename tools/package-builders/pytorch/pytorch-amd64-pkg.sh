#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
TORCH_VERSION="2.6.0"
TORCH_CUDA_VERSION="cu124"

# Build Arguments
FORCE_BUILD=false
DOWNLOAD_LATEST=false
CHECK_PACKAGE=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --force|-f)
            FORCE_BUILD=true
            shift
            ;;
        --download-latest|-d)
            DOWNLOAD_LATEST=true
            shift
            ;;
        --check|-c)
            CHECK_PACKAGE=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/pytorch/amd64/pytorch_${TORCH_VERSION}_amd64.deb"

# Download the latest version
if [[ "$DOWNLOAD_LATEST" == true ]]; then
    echo "Downloading the latest version..."
    
    # Cleanup the download directory
    rm -rf /tmp/pkg
    rm -rf /tmp/pytorch
    mkdir -p /tmp/pkg/deb

    # Download the package from the repository
    curl -L $FILE_URL --output /tmp/pkg/deb/pytorch_${TORCH_VERSION}_amd64.deb

    # Exit the script
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
    exit 0
fi

# Check if the file exists
if [[ "$FORCE_BUILD" == false ]] && curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${TORCH_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
    exit 0
else
    if [[ "$CHECK_PACKAGE" == true ]]; then
        echo "Package version ${FFMPEG_VERSION} does not exist in the repository. We're in check mode, so we're exiting with status 1."
        echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
        exit 1
    else
        echo "Package version ${TORCH_VERSION} does not exist in the repository. Building the package."
        echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
        
        # Delete Old Packages
        rm -rf /tmp/pkg
        rm -rf /tmp/libtorch

        # Create Package Directory
        mkdir -p /tmp/pkg/pytorch_${TORCH_VERSION}_amd64/usr/local
        mkdir -p /tmp/pkg/pytorch_${TORCH_VERSION}_amd64/DEBIAN

        # Create Control File
        {
            echo "Package: pytorch-mrdt"
            echo "Version: ${TORCH_VERSION}"
            echo "Maintainer: pytorch"
            echo "Depends:"
            echo "Architecture: amd64"
            echo "Homepage: https://pytorch.org/cppdocs/"
            echo "Description: A prebuilt version of Torch. Made by the Mars Rover Design Team."
        } > /tmp/pkg/pytorch_${TORCH_VERSION}_amd64/DEBIAN/control

        # Download Torch
        wget -O torch.zip https://download.pytorch.org/libtorch/${TORCH_CUDA_VERSION}/libtorch-cxx11-abi-shared-with-deps-${TORCH_VERSION}%2B${TORCH_CUDA_VERSION}.zip
        unzip torch.zip
        # rm torch.zip
        cd libtorch

        # Install Torch
        mkdir -p /tmp/pkg/pytorch_${TORCH_VERSION}_amd64/usr/include
        mkdir -p /tmp/pkg/pytorch_${TORCH_VERSION}_amd64/usr/lib
        mkdir -p /tmp/pkg/pytorch_${TORCH_VERSION}_amd64/usr/share
        cp -r /tmp/libtorch/include/* /tmp/pkg/pytorch_${TORCH_VERSION}_amd64/usr/include/
        cp -r /tmp/libtorch/lib/* /tmp/pkg/pytorch_${TORCH_VERSION}_amd64/usr/lib/
        cp -r /tmp/libtorch/share/* /tmp/pkg/pytorch_${TORCH_VERSION}_amd64/usr/share/


        # Cleanup Install
        rm -rf libtorch

        # Create Package
        dpkg --build /tmp/pkg/pytorch_${TORCH_VERSION}_amd64

        # Create Package Directory
        mkdir -p /tmp/pkg/deb

        # Copy Package
        cp /tmp/pkg/pytorch_${TORCH_VERSION}_amd64.deb /tmp/pkg/deb/pytorch_${TORCH_VERSION}_amd64.deb
    fi
fi
