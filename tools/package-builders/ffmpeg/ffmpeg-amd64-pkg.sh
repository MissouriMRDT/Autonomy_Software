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
    
    # Install Dependencies
    apt update
    apt install -y \
        libaom-dev \
        libass-dev \
        libfdk-aac-dev \
        libdav1d-dev \
        libmp3lame-dev \
        libopus-dev \
        libvorbis-dev \
        libvpx-dev \
        libx264-dev \
        libx265-dev

    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/ffmpeg

    # Create Package Directory
    mkdir -p /tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64/usr/local
    mkdir -p /tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64/DEBIAN

    # Create Control File
    {
        echo "Package: ffmpeg-mrdt"
        echo "Version: ${FFMPEG_VERSION}"
        echo "Maintainer: ffmpeg"
        echo "Depends:"
        echo "Architecture: amd64"
        echo "Homepage: https://github.com/FFmpeg/FFmpeg"
        echo "Description: A prebuilt version of ffmpeg. Made by the Mars Rover Design Team."
    } > /tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64/DEBIAN/control

    # This is a workaround for the libsvtav1-dev package not being available in the repository. The package is installed manually.
    git clone --depth=1 https://gitlab.com/AOMediaCodec/SVT-AV1.git
    cd SVT-AV1
    cd Build
    # We need to install to system first. Then we can install to the package directory.
    cmake .. -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
    make -j 8
    make install
    cmake .. -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64/usr/local
    make -j 8
    make install
    cd ../..
    rm -rf SVT-AV1

    # Download FFMPEG
    git clone --recurse-submodules --depth 1 --branch n${FFMPEG_VERSION} https://github.com/FFmpeg/FFmpeg.git ffmpeg
    cd ffmpeg

    # Configure FFMPEG
    ./configure --prefix=/tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64/usr/local \
    --enable-static \
    --disable-shared \
    --disable-doc \
    --enable-pic \
    --extra-libs="-lpthread -lm" \
    --ld="g++" \
    --enable-gpl \
    --enable-libaom \
    --enable-libass \
    --enable-libfdk-aac \
    --enable-libfreetype \
    --enable-libmp3lame \
    --enable-libopus \
    --enable-libsvtav1 \
    --enable-libdav1d \
    --enable-libvorbis \
    --enable-libvpx \
    --enable-libx264 \
    --enable-libx265 \
    --enable-nonfree

    # Install FFMPEG
    make
    make install

    # Cleanup Install
    cd ../
    rm -rf ffmpeg

    # Create Package
    dpkg --build /tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/ffmpeg_${FFMPEG_VERSION}_amd64.deb /tmp/pkg/deb/ffmpeg_${FFMPEG_VERSION}_amd64.deb
fi
