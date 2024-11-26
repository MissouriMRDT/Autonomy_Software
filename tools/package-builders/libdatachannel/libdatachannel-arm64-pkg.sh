#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
LIBDATACHANNEL_VERSION="0.22.2"

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/libdatachannel/arm64/libdatachannel_${LIBDATACHANNEL_VERSION}_arm64.deb"

# Check if the file exists
if curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${LIBDATACHANNEL_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
else
    echo "Package version ${LIBDATACHANNEL_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
    
    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/libdatachannel

    # Create Package Directory
    mkdir -p /tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_arm64/usr/local
    mkdir -p /tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_arm64/DEBIAN

    # Create Control File
    echo "Package: libdatachannel-mrdt" > /tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_arm64/DEBIAN/control
    echo "Version: ${LIBDATACHANNEL_VERSION}" >> /tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_arm64/DEBIAN/control
    echo "Maintainer: paullouisageneau" >> /tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_arm64/DEBIAN/control
    echo "Depends:" >> /tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_arm64/DEBIAN/control
    echo "Architecture: arm64" >> /tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_arm64/DEBIAN/control
    echo "Homepage: https://github.com/paullouisageneau/libdatachannel" >> /tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_arm64/DEBIAN/control
    echo "Description: A prebuilt version of libdatachannel for WebRTC and websocket connections. Made by the Mars Rover Design Team." >> /tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_arm64/DEBIAN/control

    # Download LibDataChannel
    git clone --recurse-submodules --depth 1 --branch v${LIBDATACHANNEL_VERSION} https://github.com/paullouisageneau/libdatachannel.git libdatachannel
    mkdir libdatachannel/build
    cd libdatachannel/build

    # Build LibDataChannel
    cmake \
    -D CMAKE_INSTALL_PREFIX=/tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_arm64/usr/local \
    -D USE_GNUTLS=0 -D USE_NICE=0 \
    -D BUILD_SHARED_LIBS=OFF \
    -D CMAKE_BUILD_TYPE=Release ..

    # Install LibDataChannel
    make datachannel-static
    make install datachannel-static

    # Cleanup Install
    cd ../..
    rm -rf libdatachannel

    # Create Package
    dpkg --build /tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_arm64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_arm64.deb /tmp/pkg/deb/libdatachannel_${LIBDATACHANNEL_VERSION}_arm64.deb
fi
