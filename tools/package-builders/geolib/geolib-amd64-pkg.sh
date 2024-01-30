#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
GEOLIB_VERSION="2.3"

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/geolib/amd64/geolib_${GEOLIB_VERSION}_amd64.deb"

# Check if the file exists
if curl -sI "$FILE_URL" | grep -q "HTTP/1.1 200 OK"; then
    echo "Package version ${GEOLIB_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
else
    echo "Package version ${GEOLIB_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
    
    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/geographiclib

    # Create Package Directory
    mkdir -p /tmp/pkg/geolib_${GEOLIB_VERSION}_amd64/usr/local
    mkdir -p /tmp/pkg/geolib_${GEOLIB_VERSION}_amd64/DEBIAN

    # Create Control File
    cat << EOF > /tmp/pkg/geolib_${GEOLIB_VERSION}_amd64/DEBIAN/control
Package: geographiclib-mrdt
Version: ${GEOLIB_VERSION}
Maintainer: GeographicLib
Depends: 
Architecture: amd64
Homepage: https://geographiclib.sourceforge.io/
Description: A prebuilt version of GeographicLib. Made by the Mars Rover Design Team.
EOF

    # Install GeographicLib
    git clone --depth 1 --branch v${GEOLIB_VERSION} https://github.com/geographiclib/geographiclib.git
    mkdir geographiclib/build
    cd geographiclib/build

    cmake \
    -D CMAKE_INSTALL_PREFIX=/tmp/pkg/geolib_${GEOLIB_VERSION}_amd64/usr/local \
    -D CMAKE_BUILD_TYPE=Release ..

    make
    make install

    cd ../..
    rm -rf geographiclib

    # Create Package
    dpkg --build /tmp/pkg/geolib_${GEOLIB_VERSION}_amd64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/geolib_${GEOLIB_VERSION}_amd64.deb /tmp/pkg/deb/geolib_${GEOLIB_VERSION}_amd64.deb
fi
