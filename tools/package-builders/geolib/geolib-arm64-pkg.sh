#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
GEOLIB_VERSION="v2.3"

# Create Package Directory
mkdir -p /tmp/pkg/geographiclib_${GEOLIB_VERSION}_arm64/usr/local
mkdir -p /tmp/pkg/geographiclib_${GEOLIB_VERSION}_arm64/DEBIAN

# Create Control File
cat << EOF > /tmp/pkg/geographiclib_${GEOLIB_VERSION}_arm64/DEBIAN/control
Package: geographiclib-mrdt
Version: ${GEOLIB_VERSION}
Maintainer: GeographicLib
Depends: 
Architecture: arm64
Homepage: https://geographiclib.sourceforge.io/
Description: A prebuilt version of GeographicLib. Made by the Mars Rover Design Team.
EOF

# Install GeographicLib
git clone --depth 1 --branch ${GEOLIB_VERSION} https://github.com/geographiclib/geographiclib.git
mkdir geographiclib/build
cd geographiclib/build

cmake \
-D CMAKE_BUILD_TYPE=RelWithDebInfo ..

make
make install

cd ../..
rm -rf geographiclib

# Create Package
dpkg --build /tmp/pkg/geographiclib_${GEOLIB_VERSION}_arm64

# Create Package Directory
mkdir -p /tmp/pkg/deb

# Copy Package
cp /tmp/pkg/geographiclib_${GEOLIB_VERSION}_arm64.deb /tmp/pkg/deb/geographiclib_${GEOLIB_VERSION}_arm64.deb