#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
OPENCV_VERSION="4.9.0"

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/opencv/amd64/opencv_${OPENCV_VERSION}_amd64.deb"

# Check if the file exists
if curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${OPENCV_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
else
    echo "Package version ${OPENCV_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT

    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/opencv_contrib
    rm -rf /tmp/opencv

    # Create Package Directory
    mkdir -p /tmp/pkg/opencv_${OPENCV_VERSION}_amd64/usr/local
    mkdir -p /tmp/pkg/opencv_${OPENCV_VERSION}_amd64/DEBIAN

    # Create Control File
    echo "Package: opencv-mrdt" > /tmp/pkg/opencv_${OPENCV_VERSION}_amd64/DEBIAN/control
    echo "Version: ${OPENCV_VERSION}" >> /tmp/pkg/opencv_${OPENCV_VERSION}_amd64/DEBIAN/control
    echo "Maintainer: OpenCV" >> /tmp/pkg/opencv_${OPENCV_VERSION}_amd64/DEBIAN/control
    echo "Depends:" >> /tmp/pkg/opencv_${OPENCV_VERSION}_amd64/DEBIAN/control
    echo "Architecture: amd64" >> /tmp/pkg/opencv_${OPENCV_VERSION}_amd64/DEBIAN/control
    echo "Homepage: https://opencv.org/" >> /tmp/pkg/opencv_${OPENCV_VERSION}_amd64/DEBIAN/control
    echo "Description: A prebuilt version of OpenCV with minimal packages and Cuda support. Made by the Mars Rover Design Team." >> /tmp/pkg/opencv_${OPENCV_VERSION}_amd64/DEBIAN/control


    # Download OpenCV
    git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv.git
    git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git
    mkdir opencv/build
    cd opencv/build

    # Build OpenCV
    cmake \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/tmp/pkg/opencv_${OPENCV_VERSION}_amd64/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D BUILD_SHARED_LIBS=OFF \
    -D WITH_CSTRIPES=ON \
    -D WITH_OPENCL=ON \
    -D WITH_CUDA=ON \
    -D WITH_CUDNN=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D CUDA_ARCH_BIN=7.0 \
    -D WITH_TBB=ON \
    -D WITH_OPENMP=ON \
    -D WITH_IPP=ON \
    -D WITH_CUBLAS=1 \
    -D WITH_FFMPEG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules/aruco \
    -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules/cudev \
    -D HAVE_opencv_python3=ON ..

    # Install OpenCV
    cat /proc/cpuinfo | grep "processor" | wc -l | xargs make -j
    make install
    ldconfig

    # Cleanup Install
    cd ../..
    rm -rf opencv_contrib
    rm -rf opencv

    # Create Package
    dpkg --build /tmp/pkg/opencv_${OPENCV_VERSION}_amd64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/opencv_${OPENCV_VERSION}_amd64.deb /tmp/pkg/deb/opencv_${OPENCV_VERSION}_amd64.deb
fi
