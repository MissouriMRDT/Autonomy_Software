#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
FFMPEG_VERSION="7.1"

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/ffmpeg/amd64/ffmpeg_${FFMPEG_VERSION}_amd64.deb"

# Check if the file exists
if curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${FFMPEG_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
else
    echo "Package version ${FFMPEG_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT

    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/ffmpeg

    # Create Package Directory
    mkdir -p /tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64/usr/local
    mkdir -p /tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64/DEBIAN

    # Create Control File
    echo "Package: ffmpeg-mrdt" > /tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64/DEBIAN/control
    echo "Version: ${FFMPEG_VERSION}" >> /tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64/DEBIAN/control
    echo "Maintainer: FFmpeg" >> /tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64/DEBIAN/control
    echo "Depends:" >> /tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64/DEBIAN/control
    echo "Architecture: amd64" >> /tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64/DEBIAN/control
    echo "Homepage: https://ffmpeg.org/" >> /tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64/DEBIAN/control
    echo "Description: A prebuilt version of FFmpeg. Made by the Mars Rover Design Team." >> /tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64/DEBIAN/control

    # Download FFmpeg
    git clone --depth 1 --branch n${FFMPEG_VERSION} https://git.ffmpeg.org/ffmpeg.git
    cd ffmpeg

    # Build FFmpeg
    ./configure --prefix=/tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64/usr/local --enable-gpl --enable-libx264

    # Install FFmpeg
    make -j$(nproc)
    make install
    ldconfig

    # Cleanup Install
    cd ../..
    rm -rf ffmpeg

    # Create Package
    dpkg --build /tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64.deb /tmp/pkg/deb/ffmpeg_${FFMPEG_VERSION}_amd64.deb
fi
