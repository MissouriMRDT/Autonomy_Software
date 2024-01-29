#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
GEOLIB_VERSION="2.3"

# Delete Old Packages
rm -rf /tmp/pkg

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