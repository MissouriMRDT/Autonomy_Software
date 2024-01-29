#!/bin/bash

# Set Working Directory
cd /tmp

# Fix Permissions
chmod 1777 /etc/

# Install Variables
OPENCV_VERSION="4.8.1"

# Create Package Directory
mkdir -p /tmp/pkg/opencv_${OPENCV_VERSION}_amd64/usr/local
mkdir -p /tmp/pkg/opencv_${OPENCV_VERSION}_amd64/DEBIAN

# Create Control File
cat << EOF > /tmp/pkg/opencv_${OPENCV_VERSION}_amd64/DEBIAN/control
Package: opencv-cuda
Version: ${OPENCV_VERSION}
Maintainer: OpenCV
Depends: 
Architecture: amd64
Homepage: https://opencv.org/
Description: A prebuilt version of OpenCV with minimal packages and Cuda support. Made by the Mars Rover Design Team.
EOF

# Install OpenCV
git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv.git
git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git
mkdir opencv/build
cd opencv/build

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

cat /proc/cpuinfo | grep "processor" | wc -l | xargs make -j
sudo make install
ldconfig

cd ../..
rm -rf opencv_contrib
rm -rf opencv

# Create Package
dpkg --build /tmp/pkg/dep/opencv_${OPENCV_VERSION}_amd64
