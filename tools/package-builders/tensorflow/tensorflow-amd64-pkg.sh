#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
TENSORFLOW_VERSION="2.13.1"
TENSORFLOW_COMMIT="f841394b1b714c5cc5366536411cf146c8c570df"
TENSORFLOW_COMMIT_MD5_HASH="fa01678847283115e0b359ebb4db427ab88e289ab0b20376e1a2b3cb775eb720"
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
    apt update && apt install -y bazel-${TENSORFLOW_BAZEL_VERSION} python3 python-is-python3
    # Symbolically link bazel-(version) install to /usr/bin/bazel.
    ln -fs /usr/bin/bazel-${TENSORFLOW_BAZEL_VERSION} /usr/bin/bazel
    
    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/tensorflow
    rm -rf /tmp/libedgetpu

    # Create Package Directory for Tensorflow.
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/usr/local
    mkdir -p /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN
    # Create Package Directory for Flatbuffers.
    mkdir -p /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/usr/local
    mkdir -p /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/DEBIAN
    # Create Package Directory for LibEdgeTPU.
    mkdir -p /tmp/pkg/libedgetpu_${TENSORFLOW_VERSION}_amd64/usr/local
    mkdir -p /tmp/pkg/libedgetpu_${TENSORFLOW_VERSION}_amd64/DEBIAN

    # Create Control File for Tensorflow.
    echo "Package: tensorflow-mrdt" > /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Version: ${TENSORFLOW_VERSION}" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Maintainer: google" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Depends:" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Architecture: amd64" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Homepage: https://www.tensorflow.org/api_docs/cc" >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Description: A prebuilt version of Tensorflow. Made by the Mars Rover Design Team." >> /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    # Create Control File for Flatbuffers.
    echo "Package: flatbuffers-mrdt" > /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Version: ${TENSORFLOW_VERSION}" >> /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Maintainer: google" >> /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Depends:" >> /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Architecture: amd64" >> /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Homepage: https://flatbuffers.dev/flatbuffers_guide_use_cpp.html" >> /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Description: A prebuilt version of Flatbuffers. Made by the Mars Rover Design Team." >> /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    # Create Control File for LibEdgeTPU.
    echo "Package: libedgetpu-mrdt" > /tmp/pkg/libedgetpu_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Version: ${TENSORFLOW_VERSION}" >> /tmp/pkg/libedgetpu_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Maintainer: google" >> /tmp/pkg/libedgetpu_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Depends:" >> /tmp/pkg/libedgetpu_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Architecture: amd64" >> /tmp/pkg/libedgetpu_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Homepage: https://github.com/google-coral/libedgetpu" >> /tmp/pkg/libedgetpu_${TENSORFLOW_VERSION}_amd64/DEBIAN/control
    echo "Description: A prebuilt version of LibEdgeTPU. Made by the Mars Rover Design Team." >> /tmp/pkg/libedgetpu_${TENSORFLOW_VERSION}_amd64/DEBIAN/control

    # Download Tensorflow
    git clone --depth 1 --branch v${TENSORFLOW_VERSION} https://github.com/tensorflow/tensorflow
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
        -D CMAKE_INSTALL_PREFIX=/tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/usr/local \
        -G "Unix Makefiles" \
        -D CMAKE_BUILD_TYPE=Release ..

    # Install Flatbuffers
    make
    make install
    mkdir -p /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/usr/lib
    cp -r /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/usr/local/lib/*flatbuffers* /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64/usr/lib/

    # Cleanup Install
    cd /tmp
    rm -rf tensorflow

    # Download LibEdgeTPU
    git clone --depth 1 https://github.com/google-coral/libedgetpu.git
    cd libedgetpu

    # Build LibEdgeTPU
    sed -i 's/TENSORFLOW_COMMIT = "[^"]*"/TENSORFLOW_COMMIT = "'"${TENSORFLOW_COMMIT}"'"/' ./workspace.bzl
    sed -i 's/TENSORFLOW_SHA256 = "[^"]*"/TENSORFLOW_SHA256 = "'"${TENSORFLOW_COMMIT_MD5_HASH}"'"/' ./workspace.bzl
    make

    # Install LibEdgeTPU
    mkdir -p /tmp/pkg/libedgetpu_${TENSORFLOW_VERSION}_amd64/usr/local/lib/ && cp ./out/direct/k8/libedgetpu.so.1.0 /tmp/pkg/libedgetpu_${TENSORFLOW_VERSION}_amd64/usr/local/lib/
    mkdir -p /tmp/pkg/libedgetpu_${TENSORFLOW_VERSION}_amd64/usr/local/include/edgetpu/ && cp ./tflite/public/edgetpu.h /tmp/pkg/libedgetpu_${TENSORFLOW_VERSION}_amd64/usr/local/include/edgetpu/

    # # Cleanup Install
    # cd ../
    # rm -rf libedgetpu

    # Create Package
    dpkg --build /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64
    dpkg --build /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64
    dpkg --build /tmp/pkg/libedgetpu_${TENSORFLOW_VERSION}_amd64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/tensorflow_${TENSORFLOW_VERSION}_amd64.deb /tmp/pkg/deb/tensorflow_${TENSORFLOW_VERSION}_amd64.deb
    cp /tmp/pkg/flatbuffers_${TENSORFLOW_VERSION}_amd64.deb /tmp/pkg/deb/flatbuffers_${TENSORFLOW_VERSION}_amd64.deb
    cp /tmp/pkg/libedgetpu_${TENSORFLOW_VERSION}_amd64.deb /tmp/pkg/deb/libedgetpu_${TENSORFLOW_VERSION}_amd64.deb
fi
