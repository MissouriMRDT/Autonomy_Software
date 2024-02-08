#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
TENSORFLOW_VERSION="2.13.1"

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/tensorflow/amd64/tensorflow_${TENSORFLOW_VERSION}_amd64.deb"

# Check if the file exists
if curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${TENSORFLOW_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
else
    echo "Package version ${TENSORFLOW_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
    
    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/tensorflow

    # Create Package Directory for Tensorflow.
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN
    # # Create Package Directory for Flatbuffers.
    # mkdir -p /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/usr/local
    # mkdir -p /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/DEBIAN

    # Create Control File for Tensorflow.
    echo "Package: tensorflow-mrdt" > /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Version: ${TENSORFLOW_VERSION}" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Maintainer: odygrd" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Depends:" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Architecture: amd64" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Homepage: https://www.tensorflow.org/api_docs/cc" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Description: A prebuilt version of Tensorflow. Made by the Mars Rover Design Team." >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    # # Create Control File for Flatbuffers.
    # echo "Package: flatbuffers-mrdt" > /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    # echo "Version: ${TENSORFLOW_VERSION}" >> /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    # echo "Maintainer: odygrd" >> /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    # echo "Depends:" >> /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    # echo "Architecture: amd64" >> /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    # echo "Homepage: https://flatbuffers.dev/flatbuffers_guide_use_cpp.html" >> /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    # echo "Description: A prebuilt version of Tensorflow. Made by the Mars Rover Design Team." >> /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/DEBIAN/control

    # Download Tensorflow
    git clone --depth 1 --branch v${TENSORFLOW_VERSION} https://github.com/tensorflow/tensorflow

    # Build Tensorflow
    mkdir tensorflow/tensorflow/lite/build && cd tensorflow/tensorflow/lite/build
    cmake \
    -D CMAKE_INSTALL_PREFIX=/tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local \
    -D CMAKE_BUILD_TYPE=Release ..

    # Install Tensorflow
    make
    make install
    cp libtensorflow-lite.a /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local/lib/
    mkdir /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local/include/tensorflow
    cp -r ../../lite /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local/include/tensorflow/lite

    # # Build Flatbuffers
    # mkdir flatbuffers/build && cd flatbuffers/build
    # cmake \
    #     -D CMAKE_INSTALL_PREFIX=/tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/usr/local \
    #     -D CMAKE_BUILD_TYPE=Release ..

    # # Install Flatbuffers
    # make
    # make install

    # Cleanup Install
    # cd ../../../../
    # rm -rf tensorflow

    # Create Package
    dpkg --build /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64.deb /tmp/pkg/deb/tensorflow_${TENSORFLOW_VERSION}_amd64.deb
fi
