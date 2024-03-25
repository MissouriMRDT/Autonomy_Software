#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
TENSORFLOW_VERSION="2.13.1"
TENSORFLOW_COMMIT="f841394b1b714c5cc5366536411cf146c8c570df"
TENSORFLOW_COMMIT_MD5_HASH="fa01678847283115e0b359ebb4db427ab88e289ab0b20376e1a2b3cb775eb720"
TENSORFLOW_BAZEL_VERSION="5.3.0"

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/tensorflow/arm64/tensorflow_${TENSORFLOW_VERSION}_arm64.deb"

# Check if the file exists
if curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${TENSORFLOW_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
else
    echo "Package version ${TENSORFLOW_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT

    # Install BAZEL build system.
    wget https://github.com/bazelbuild/bazelisk/releases/download/v1.19.0/bazelisk-linux-arm64
    chmod +x bazelisk-linux-arm64 && mv bazelisk-linux-arm64 /usr/local/bin/bazel

    # Install python packages.
    apt update && apt install -y python3 python-is-python3
    
    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/tensorflow
    rm -rf /tmp/libedgetpu

    # Create Package Directory for Tensorflow.
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/usr/local
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/DEBIAN

    # Create Control File for Tensorflow.
    echo "Package: tensorflow-mrdt" > /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/DEBIAN/control
    echo "Version: ${TENSORFLOW_VERSION}" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/DEBIAN/control
    echo "Maintainer: google" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/DEBIAN/control
    echo "Depends:" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/DEBIAN/control
    echo "Architecture: arm64" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/DEBIAN/control
    echo "Homepage: https://www.tensorflow.org/api_docs/cc" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/DEBIAN/control
    echo "Description: A prebuilt version of Tensorflow. Made by the Mars Rover Design Team." >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/DEBIAN/control

    # Download Tensorflow
    git clone --depth 1 --recurse-submodules --branch v${TENSORFLOW_VERSION} https://github.com/tensorflow/tensorflow
    cd tensorflow

    # Fix CUDA Library Path
    ln -fs /usr/local/cuda-11.4/targets/aarch64-linux/lib/stubs_/ /usr/local/cuda-11.4/targets/aarch64-linux/lib/stubs

    # Build Tensorflow
    ldconfig && USE_BAZEL_VERSION="${TENSORFLOW_BAZEL_VERSION}" bazel build \
        -c opt \
        --config=elinux_aarch64 \
        --config=monolithic \
        --config=cuda \
        --config=tensorrt \
        --verbose_failures \
        --action_env TF_CUDA_COMPUTE_CAPABILITIES="8.7" \
        --jobs 1 \
        --local_ram_resources=HOST_RAM*0.2 \
        --local_cpu_resources=HOST_CPUS*0.2 \
        //tensorflow/lite:libtensorflowlite.so

    # Install Tensorflow
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/usr/local/lib/ && cp bazel-bin/tensorflow/lite/libtensorflowlite.so /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/usr/local/lib/
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/usr/local/include/tensorflow/lite && cp -r tensorflow/lite /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/usr/local/include/tensorflow/

    # Build Flatbuffers
    mkdir -p /tmp/tensorflow/bazel-tensorflow/external/flatbuffers/build && cd /tmp/tensorflow/bazel-tensorflow/external/flatbuffers/build
    cmake \
        -D CMAKE_INSTALL_PREFIX=/tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/usr/local \
        -G "Unix Makefiles" \
        -D CMAKE_BUILD_TYPE=Release ..

    # Install Flatbuffers
    make
    make install
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/usr/lib
    cp -r /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/usr/local/lib/*flatbuffers* /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/usr/lib/

    # Cleanup Install
    cd /tmp
    rm -rf tensorflow

    # Download LibEdgeTPU
    git clone --recurse-submodules https://github.com/google-coral/libedgetpu.git
    cd libedgetpu
    git checkout -f a82c669fb7a9b2e813cfb3d5409fea98d6a6ac8c

    # Build LibEdgeTPU
    sed -i 's/TENSORFLOW_COMMIT = "[^"]*"/TENSORFLOW_COMMIT = "'"${TENSORFLOW_COMMIT}"'"/' ./workspace.bzl
    sed -i 's/TENSORFLOW_SHA256 = "[^"]*"/TENSORFLOW_SHA256 = "'"${TENSORFLOW_COMMIT_MD5_HASH}"'"/' ./workspace.bzl
    make CPU="aarch64"

    # Install LibEdgeTPU
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/usr/local/lib/ && cp ./out/direct/aarch64/libedgetpu.so.1.0 /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/usr/local/lib/
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/usr/local/include/edgetpu/ && cp ./tflite/public/edgetpu.h /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64/usr/local/include/edgetpu/

    # Cleanup Install
    cd ../
    rm -rf libedgetpu

    # Create Package
    dpkg --build /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_arm64.deb /tmp/pkg/deb/tensorflow_${TENSORFLOW_VERSION}_arm64.deb
fi
