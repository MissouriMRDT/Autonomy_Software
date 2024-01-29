#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
QUILL_VERSION="3.6.0"

# Create Package Directory
mkdir -p /tmp/pkg/quill_${QUILL_VERSION}_arm64/usr/local
mkdir -p /tmp/pkg/quill_${QUILL_VERSION}_arm64/DEBIAN

# Create Control File
cat << EOF > /tmp/pkg/quill_${QUILL_VERSION}_arm64/DEBIAN/control
Package: quill-mrdt
Version: ${QUILL_VERSION}
Maintainer: odygrd
Depends: 
Architecture: arm64
Homepage: https://quillcpp.readthedocs.io/en/latest/
Description: A prebuilt version of Quill. Made by the Mars Rover Design Team.
EOF

# Install Google Test
git clone --depth 1 --branch v${QUILL_VERSION} http://github.com/odygrd/quill.git
mkdir quill/build
cd quill/build

cmake \
-D CMAKE_INSTALL_PREFIX=/tmp/pkg/quill_${QUILL_VERSION}_arm64/usr/local ..

make
make install

cd ../..
rm -rf quill

# Create Package
dpkg --build /tmp/pkg/quill_${QUILL_VERSION}_arm64

# Create Package Directory
mkdir -p /tmp/pkg/deb

# Copy Package
cp /tmp/pkg/quill_${QUILL_VERSION}_arm64.deb /tmp/pkg/deb/quill_${QUILL_VERSION}_arm64.deb