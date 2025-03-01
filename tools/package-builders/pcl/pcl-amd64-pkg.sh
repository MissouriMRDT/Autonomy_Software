#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
PCL_VERSION="1.15.0"

# Build Arguments
FORCE_BUILD=false
DOWNLOAD_LATEST=false

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
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/pcl/amd64/pcl_${PCL_VERSION}_amd64.deb"

# Download the latest version
if [[ "$DOWNLOAD_LATEST" == true ]]; then
    echo "Downloading the latest version..."
    
    # Cleanup the download directory
    rm -rf /tmp/pkg
    rm -rf /tmp/pcl
    mkdir -p /tmp/pkg/deb

    # Download the package from the repository
    curl -L $FILE_URL --output /tmp/pkg/deb/pcl_${PCL_VERSION}_amd64.deb

    # Exit the script
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
    exit 0
fi

# Check if the file exists
if [[ "$FORCE_BUILD" == false ]] && curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${PCL_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
else
    echo "Package version ${PCL_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
    
    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/pcl

    # Create Package Directory
    mkdir -p /tmp/pkg/pcl_${PCL_VERSION}_amd64/usr/local
    mkdir -p /tmp/pkg/pcl_${PCL_VERSION}_amd64/DEBIAN

    # Create Control File
    {
        echo "Package: pcl-mrdt"
        echo "Version: ${PCL_VERSION}"
        echo "Maintainer: PointCloudLibrary"
        echo "Depends:"
        echo "Architecture: amd64"
        echo "Homepage: https://github.com/PointCloudLibrary/pcl"
        echo "Description: A prebuilt version of pcl. Made by the Mars Rover Design Team."
    } > /tmp/pkg/pcl_${PCL_VERSION}_amd64/DEBIAN/control

    # Download LibDataChannel
    git clone --recurse-submodules --depth 1 --branch pcl-${PCL_VERSION} https://github.com/PointCloudLibrary/pcl.git pcl
    mkdir pcl/build
    cd pcl/build

    # Build LibDataChannel
    cmake \
    -D CMAKE_INSTALL_PREFIX=/tmp/pkg/pcl_${PCL_VERSION}_amd64/usr/local \
    -D CMAKE_BUILD_TYPE=Release ..

    # Install LibDataChannel
    make
    make install

    # Cleanup Install
    cd ../..
    rm -rf pcl

    # Create Package
    dpkg --build /tmp/pkg/pcl_${PCL_VERSION}_amd64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/pcl_${PCL_VERSION}_amd64.deb /tmp/pkg/deb/pcl_${PCL_VERSION}_amd64.deb
fi
