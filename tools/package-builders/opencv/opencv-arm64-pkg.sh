#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
OPENCV_VERSION="4.11.0"

# Build Arguments
FORCE_BUILD=false
DOWNLOAD_LATEST=false
CHECK_PACKAGE=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --force|-f)
            FORCE_BUILD=true
            shift
            ;;
        --download-latest|-d)
            DOWNLOAD_LATEST=true
            shift
            ;;
        --check|-c)
            CHECK_PACKAGE=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/opencv/arm64/opencv_${OPENCV_VERSION}_arm64.deb"

# Download the latest version
if [[ "$DOWNLOAD_LATEST" == true ]]; then
    echo "Downloading the latest version..."
    
    # Cleanup the download directory
    rm -rf /tmp/pkg
    rm -rf /tmp/opencv
    mkdir -p /tmp/pkg/deb

    # Download the package from the repository
    curl -L $FILE_URL --output /tmp/pkg/deb/opencv_${OPENCV_VERSION}_arm64.deb

    # Exit the script
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
    exit 0
fi

# Check if the file exists
if [[ "$FORCE_BUILD" == false ]] && curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${OPENCV_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
    exit 0
else
    if [[ "$CHECK_PACKAGE" == true ]]; then
        echo "Package version ${FFMPEG_VERSION} does not exist in the repository. We're in check mode, so we're exiting with status 1."
        echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
        exit 1
    else
        echo "Package version ${OPENCV_VERSION} does not exist in the repository or the forced flag was thrown. Building the package."
        echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT

        # Delete Old Packages
        rm -rf /tmp/pkg
        rm -rf /tmp/opencv_contrib
        rm -rf /tmp/opencv

        # Create Package Directory
        mkdir -p /tmp/pkg/opencv_${OPENCV_VERSION}_arm64/usr/local
        mkdir -p /tmp/pkg/opencv_${OPENCV_VERSION}_arm64/DEBIAN

        # Create Control File
        {
            echo "Package: opencv-mrdt"
            echo "Version: ${OPENCV_VERSION}"
            echo "Maintainer: OpenCV"
            echo "Depends:"
            echo "Architecture: arm64"
            echo "Homepage: https://opencv.org/"
            echo "Description: A prebuilt version of OpenCV with Cuda support. Made by the Mars Rover Design Team."
        } > /tmp/pkg/opencv_${OPENCV_VERSION}_arm64/DEBIAN/control

        # Download OpenCV
        git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv.git
        git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git
        mkdir opencv/build
        cd opencv/build

        # Build OpenCV
        cmake \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/tmp/pkg/opencv_${OPENCV_VERSION}_arm64/usr/local \
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

        # Install OpenCV
        cat /proc/cpuinfo | grep "processor" | wc -l | xargs make -j
        make install
        ldconfig

        # Cleanup Install
        cd ../..
        rm -rf opencv_contrib
        rm -rf opencv

        # Create Package
        dpkg --build /tmp/pkg/opencv_${OPENCV_VERSION}_arm64

        # Create Package Directory
        mkdir -p /tmp/pkg/deb

        # Copy Package
        cp /tmp/pkg/opencv_${OPENCV_VERSION}_arm64.deb /tmp/pkg/deb/opencv_${OPENCV_VERSION}_arm64.deb
    fi
fi
