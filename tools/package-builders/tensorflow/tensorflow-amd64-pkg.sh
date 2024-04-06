#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
TENSORFLOW_VERSION="2.15.0"
TENSORFLOW_COMMIT="6887368d6d46223f460358323c4b76d61d1558a8"
TENSORFLOW_COMMIT_MD5_HASH="bb25fa4574e42ea4d452979e1d2ba3b86b39569d6b8106a846a238b880d73652"
TENSORFLOW_BAZEL_VERSION="6.1.0"

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/tensorflow/amd64/tensorflow_${TENSORFLOW_VERSION}_amd64.deb"

# Check if the file exists
if curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${TENSORFLOW_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
else
    echo "Package version ${TENSORFLOW_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT

    # Install BAZEL build system.
    curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor >bazel-archive-keyring.gpg
    mv bazel-archive-keyring.gpg /usr/share/keyrings
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list
    apt update && apt install -y bazel-${TENSORFLOW_BAZEL_VERSION} python3 python-is-python3 libabsl-dev libflatbuffers-dev
    # Symbolically link bazel-(version) install to /usr/bin/bazel.
    ln -fs /usr/bin/bazel-${TENSORFLOW_BAZEL_VERSION} /usr/bin/bazel

    # Install Docker in Docker. Using docker to build libedgetpu is by far the easiest way to do this.
    if ! command -v docker &> /dev/null; then
        # Docker is not installed, proceed with installation
        echo "Installing Docker..."
        curl -sSL https://get.docker.com/ | sh
        ulimit -n 65536 in /etc/init.d/docker
        service docker start
    else
        # Docker is already installed
        echo "Docker is already installed."
    fi
    
    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/tensorflow
    rm -rf /tmp/libedgetpu

    # Create Package Directory for Tensorflow AMD64.
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN

    # Create Control File for Tensorflow AMD64.
    echo "Package: tensorflow-mrdt" > /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Version: ${TENSORFLOW_VERSION}" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Maintainer: google" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Depends:" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Architecture: amd64" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Homepage: https://www.tensorflow.org/api_docs/cc" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Description: A prebuilt version of Tensorflow. Made by the Mars Rover Design Team." >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control

    # Download Tensorflow
    git clone --depth 1 --recurse-submodules --branch v${TENSORFLOW_VERSION} https://github.com/tensorflow/tensorflow
    cd tensorflow

    # Build Tensorflow
    bazel build \
        -c opt \
        --config=monolithic \
        --config=cuda \
        --config=tensorrt \
        --verbose_failures \
        --local_ram_resources=HOST_RAM*0.5 \
        --local_cpu_resources=HOST_CPUS*0.5 \
        //tensorflow/lite:libtensorflowlite.so

    # Install Tensorflow
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local/lib/ && cp bazel-bin/tensorflow/lite/libtensorflowlite.so /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local/lib/
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local/include/tensorflow/lite && cp -r tensorflow/lite /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local/include/tensorflow/

    # Build Flatbuffers
    mkdir -p /tmp/tensorflow/bazel-tensorflow/external/flatbuffers/build && cd /tmp/tensorflow/bazel-tensorflow/external/flatbuffers/build
    cmake \
        -D CMAKE_INSTALL_PREFIX=/tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local \
        -G "Unix Makefiles" \
        -D CMAKE_BUILD_TYPE=Release ..

    # Install Flatbuffers
    make
    make install
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/lib
    cp -r /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local/lib/*flatbuffers* /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/lib/

    # Cleanup Install
    rm -rf /tmp/tensorflow

    # Download LibEdgeTPU
    cd /tmp/ && git clone --recurse-submodules https://github.com/google-coral/libedgetpu.git
    cd libedgetpu

    # sed -i '/^DOCKER_MAKE_COMMAND :=/c DOCKER_MAKE_COMMAND := \\\necho $(MAKEFILE_DIR); \\' Makefile
    # Replace docker workspace in Makefile with our docker volume.
    # sed -i 's/DOCKER_CONTEXT_DIR := $(MAKEFILE_DIR)/DOCKER_CONTEXT_DIR = /workspace/docker/g' ./Makefile
    # sed -i 's/DOCKER_WORKSPACE := $(MAKEFILE_DIR)/DOCKER_WORKSPACE = libedgetpu_build/g' ./Makefile
    # sed -i '/--embed_label/d; /--stamp/ i \\  --embed_label="'${TENSORFLOW_COMMIT}'" --experimental_repo_remote_exec \\' ./Makefile
    # Set LibEdgeTPU tensorflow commit.
    sed -i 's/TENSORFLOW_COMMIT = "[^"]*"/TENSORFLOW_COMMIT = "'"${TENSORFLOW_COMMIT}"'"/' ./workspace.bzl
    sed -i 's/TENSORFLOW_SHA256 = "[^"]*"/TENSORFLOW_SHA256 = "'"${TENSORFLOW_COMMIT_MD5_HASH}"'"/' ./workspace.bzl

    # # Create external docker volume for sharing libedgetpu repo.
    # docker volume create libedgetpu_build
    # # Volume must be tied to dummy container to work.
    # docker container create --name dummy -v libedgetpu_build:/workspace alpine
    # # Iterate through files and directories in /tmp/libedgetpu
    # for FILE_OR_DIR in /tmp/libedgetpu/*; do
    #     # Extract the file or directory name
    #     FILE_NAME=$(basename "$FILE_OR_DIR")
    #     # Copy the file or directory into the container
    #     docker cp "$FILE_OR_DIR" "dummy:/workspace/$FILE_NAME"
    # done

    # Build LibEdgeTPU.
    DOCKER_CPUS="k8" DOCKER_IMAGE="ubuntu:22.04" DOCKER_TARGETS=libedgetpu make docker-build
    # Copy build out dir from docker volume to current dir.
    # docker cp libedgetpu-builder:/workspace/out ./

    # Install LibEdgeTPU
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local/lib/ && mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local/include/edgetpu/
    cp ./out/direct/k8/libedgetpu.so.1.0 /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local/lib/
    cp ./tflite/public/edgetpu.h /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local/include/edgetpu/


    # Cleanup Install
    rm -rf /tmp/libedgetpu
    # Delete docker dummy container and volume.
    # docker rm dummy
    # docker volume rm libedgetpu_build

    # Create Package
    dpkg --build /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64.deb /tmp/pkg/deb/tensorflow_${TENSORFLOW_VERSION}_amd64.deb
fi
