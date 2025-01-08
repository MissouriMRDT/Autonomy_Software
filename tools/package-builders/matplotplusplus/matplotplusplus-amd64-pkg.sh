#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
MATPLOT_VERSION="master"

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/matplotplusplus/amd64/matplotplusplus_${MATPLOT_VERSION}_amd64.deb"

# Check if the file exists
if curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${MATPLOT_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
else
    echo "Package version ${MATPLOT_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
    
    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/matplotplusplus

    # Create Package Directory
    mkdir -p /tmp/pkg/matplotplusplus_${MATPLOT_VERSION}_amd64/usr/local
    mkdir -p /tmp/pkg/matplotplusplus_${MATPLOT_VERSION}_amd64/DEBIAN

    # Create Control File
    {
        echo "Package: matplotplusplus-mrdt"
        echo "Version: 1.6.0"
        echo "Maintainer: alandefreitas"
        echo "Depends:"
        echo "Architecture: amd64"
        echo "Homepage: https://github.com/alandefreitas/matplotplusplus.git"
        echo "Description: A prebuilt version of matplotplusplus for WebRTC and websocket connections. Made by the Mars Rover Design Team."
    } > /tmp/pkg/matplotplusplus_${MATPLOT_VERSION}_amd64/DEBIAN/control

    # Download LibDataChannel
    git clone --recurse-submodules --depth 1 --branch ${MATPLOT_VERSION} https://github.com/alandefreitas/matplotplusplus.git matplotplusplus
    mkdir matplotplusplus/build
    cd matplotplusplus/build

    # Build LibDataChannel
    cmake \
    -D CMAKE_INSTALL_PREFIX=/tmp/pkg/matplotplusplus_${MATPLOT_VERSION}_amd64/usr/local \
    -D CMAKE_BUILD_TYPE=Release ..

    # Install LibDataChannel
    make
    make install

    # Cleanup Install
    cd ../..
    rm -rf matplotplusplus

    # Create Package
    dpkg --build /tmp/pkg/matplotplusplus_${MATPLOT_VERSION}_amd64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/matplotplusplus_${MATPLOT_VERSION}_amd64.deb /tmp/pkg/deb/matplotplusplus_${MATPLOT_VERSION}_amd64.deb
fi
