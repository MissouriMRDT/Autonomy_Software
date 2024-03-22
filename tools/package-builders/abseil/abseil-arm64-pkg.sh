#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
ABSEIL_VERSION="20230802.1"

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/abseil/arm64/abseil_${ABSEIL_VERSION}_arm64.deb"

# Check if the file exists
if curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${ABSEIL_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
else
    echo "Package version ${ABSEIL_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
    
    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/abseil

    # Create Package Directory
    mkdir -p /tmp/pkg/abseil_${ABSEIL_VERSION}_arm64/usr/local
    mkdir -p /tmp/pkg/abseil_${ABSEIL_VERSION}_arm64/DEBIAN

    # Create Control File
    echo "Package: abseil-mrdt" > /tmp/pkg/abseil_${ABSEIL_VERSION}_arm64/DEBIAN/control
    echo "Version: ${ABSEIL_VERSION}" >> /tmp/pkg/abseil_${ABSEIL_VERSION}_arm64/DEBIAN/control
    echo "Maintainer: abseil" >> /tmp/pkg/abseil_${ABSEIL_VERSION}_arm64/DEBIAN/control
    echo "Depends:" >> /tmp/pkg/abseil_${ABSEIL_VERSION}_arm64/DEBIAN/control
    echo "Architecture: arm64" >> /tmp/pkg/abseil_${ABSEIL_VERSION}_arm64/DEBIAN/control
    echo "Homepage: https://abseil.io/docs/cpp/guides/" >> /tmp/pkg/abseil_${ABSEIL_VERSION}_arm64/DEBIAN/control
    echo "Description: A prebuilt version of Abseil. Made by the Mars Rover Design Team." >> /tmp/pkg/abseil_${ABSEIL_VERSION}_arm64/DEBIAN/control

    # Download Abseil
    git clone --depth 1 --branch ${ABSEIL_VERSION} https://github.com/abseil/abseil-cpp.git
    mkdir -p abseil-cpp/build && cd abseil-cpp/build

    # Build Abseil
    cmake \
    -D CMAKE_INSTALL_PREFIX=/tmp/pkg/abseil_${ABSEIL_VERSION}_arm64/usr/local \
    -D CMAKE_BUILD_TYPE=Release \
    -D ABSL_ENABLE_INSTALL=ON ..

    # Install Abseil
    make
    make install
    mkdir -p /tmp/pkg/abseil_${ABSEIL_VERSION}_arm64/usr/lib
    cp -r /tmp/pkg/abseil_${ABSEIL_VERSION}_arm64/usr/local/lib/libabsl* /tmp/pkg/abseil_${ABSEIL_VERSION}_arm64/usr/lib/

    # Cleanup Install
    cd ../..
    rm -rf abseil

    # Create Package
    dpkg --build /tmp/pkg/abseil_${ABSEIL_VERSION}_arm64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/abseil_${ABSEIL_VERSION}_arm64.deb /tmp/pkg/deb/abseil_${ABSEIL_VERSION}_arm64.deb
fi
