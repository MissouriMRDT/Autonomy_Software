#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
OPENCV_VERSION="4.8.1"

# Delete Old Packages
rm -rf /tmp/pkg
rm -rf /tmp/opencv_contrib
rm -rf /tmp/opencv

# Create Package Directory
mkdir -p /tmp/pkg/opencv_${OPENCV_VERSION}_arm64/usr/local
mkdir -p /tmp/pkg/opencv_${OPENCV_VERSION}_arm64/DEBIAN

# Create Control File
cat << EOF > /tmp/pkg/opencv_${OPENCV_VERSION}_arm64/DEBIAN/control
Package: opencv-mrdt
Version: ${OPENCV_VERSION}
Maintainer: OpenCV
Depends: 
Architecture: arm64
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
-D BUILD_EXAMPLES=OFF \
-D BUILD_PERF_TESTS=OFF \
-D BUILD_TESTS=OFF \
-D WITH_CUDA=ON \
-D WITH_CUDNN=ON \
-D OPENCV_DNN_CUDA=ON \
-D WITH_VTK=OFF \
-D WITH_TBB=ON \
-D ENABLE_FAST_MATH=1 \
-D CUDA_FAST_MATH=1 \
-D CUDA_ARCH_PTX="" \
-D CUDA_ARCH_BIN="8.7" \
-D WITH_CUBLAS=1 \
-D WITH_FFMPEG=ON \
-D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules/aruco \
-D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules/cudev \
-D HAVE_opencv_python3=ON ..

cat /proc/cpuinfo | grep "processor" | wc -l | xargs make -j
make install
ldconfig

cd ../..
rm -rf opencv_contrib
rm -rf opencv

# Create Package
dpkg --build /tmp/pkg/opencv_${OPENCV_VERSION}_arm64

# Create Package Directory
mkdir -p /tmp/pkg/deb

# Copy Package
cp /tmp/pkg/opencv_${OPENCV_VERSION}_arm64.deb /tmp/pkg/deb/opencv_${OPENCV_VERSION}_arm64.deb