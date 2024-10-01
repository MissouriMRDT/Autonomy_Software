#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
QUILL_VERSION="7.3.0"

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/quill/amd64/quill_${QUILL_VERSION}_amd64.deb"

# Check if the file exists
if curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${QUILL_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
else
    echo "Package version ${QUILL_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
    
    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/quill

    # Create Package Directory
    mkdir -p /tmp/pkg/quill_${QUILL_VERSION}_amd64/usr/local
    mkdir -p /tmp/pkg/quill_${QUILL_VERSION}_amd64/DEBIAN

    # Create Control File
    echo "Package: quill-mrdt" > /tmp/pkg/quill_${QUILL_VERSION}_amd64/DEBIAN/control
    echo "Version: ${QUILL_VERSION}" >> /tmp/pkg/quill_${QUILL_VERSION}_amd64/DEBIAN/control
    echo "Maintainer: odygrd" >> /tmp/pkg/quill_${QUILL_VERSION}_amd64/DEBIAN/control
    echo "Depends:" >> /tmp/pkg/quill_${QUILL_VERSION}_amd64/DEBIAN/control
    echo "Architecture: amd64" >> /tmp/pkg/quill_${QUILL_VERSION}_amd64/DEBIAN/control
    echo "Homepage: https://quillcpp.readthedocs.io/en/latest/" >> /tmp/pkg/quill_${QUILL_VERSION}_amd64/DEBIAN/control
    echo "Description: A prebuilt version of Quill. Made by the Mars Rover Design Team." >> /tmp/pkg/quill_${QUILL_VERSION}_amd64/DEBIAN/control

    # Download Quill
    git clone --depth 1 --branch v${QUILL_VERSION} https://github.com/odygrd/quill.git
    mkdir quill/build
    cd quill/build

    # Build Quill
    cmake \
    -D CMAKE_INSTALL_PREFIX=/tmp/pkg/quill_${QUILL_VERSION}_amd64/usr/local \
    -D CMAKE_BUILD_TYPE=Release ..

    # Install Quill
    make
    make install

    # Cleanup Install
    cd ../..
    rm -rf quill

    # Create Package
    dpkg --build /tmp/pkg/quill_${QUILL_VERSION}_amd64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/quill_${QUILL_VERSION}_amd64.deb /tmp/pkg/deb/quill_${QUILL_VERSION}_amd64.deb
fi
