#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
GTEST_VERSION="1.14.0"

# Delete Old Packages
rm -rf /tmp/pkg
rm -rf /tmp/googletest

# Create Package Directory
mkdir -p /tmp/pkg/gtest_${GTEST_VERSION}_arm64/usr/local
mkdir -p /tmp/pkg/gtest_${GTEST_VERSION}_arm64/DEBIAN

# Create Control File
cat << EOF > /tmp/pkg/gtest_${GTEST_VERSION}_arm64/DEBIAN/control
Package: googletest-mrdt
Version: ${GTEST_VERSION}
Maintainer: Google
Depends: 
Architecture: arm64
Homepage: https://google.github.io/googletest/
Description: A prebuilt version of Google Test. Made by the Mars Rover Design Team.
EOF

# Install Google Test
git clone --depth 1 --branch v${GTEST_VERSION} https://github.com/google/googletest.git
mkdir googletest/build
cd googletest/build

cmake \
-D CMAKE_INSTALL_PREFIX=/tmp/pkg/gtest_${GTEST_VERSION}_arm64/usr/local ..

make
make install

cd ../..
rm -rf googletest

# Create Package
dpkg --build /tmp/pkg/gtest_${GTEST_VERSION}_arm64

# Create Package Directory
mkdir -p /tmp/pkg/deb

# Copy Package
cp /tmp/pkg/gtest_${GTEST_VERSION}_arm64.deb /tmp/pkg/deb/gtest_${GTEST_VERSION}_arm64.deb