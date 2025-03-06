#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
MATPLOT_VERSION="master"

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
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/matplotplusplus/amd64/matplotplusplus_${MATPLOT_VERSION}_amd64.deb"

# Download the latest version
if [[ "$DOWNLOAD_LATEST" == true ]]; then
    echo "Downloading the latest version..."
    
    # Cleanup the download directory
    rm -rf /tmp/pkg
    rm -rf /tmp/matplotplusplus
    mkdir -p /tmp/pkg/deb

    # Download the package from the repository
    curl -L $FILE_URL --output /tmp/pkg/deb/matplotplusplus_${MATPLOT_VERSION}_amd64.deb

    # Exit the script
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
    exit 0
fi

# Check if the file exists
if [[ "$FORCE_BUILD" == false ]] && curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${MATPLOT_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
    exit 0
else
    echo "Package version ${MATPLOT_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
    
    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/matplotplusplus

    # Create Package Directory
    mkdir -p /tmp/pkg/matplotplusplus_${MATPLOT_VERSION}_amd64/usr/local
    mkdir -p /tmp/pkg/matplotplusplus_${MATPLOT_VERSION}_amd64/DEBIAN

    # Create Control File
    {
        echo "Package: matplotplusplus-mrdt"
        echo "Version: 1.6.0"
        echo "Maintainer: alandefreitas"
        echo "Depends:"
        echo "Architecture: amd64"
        echo "Homepage: https://github.com/alandefreitas/matplotplusplus.git"
        echo "Description: A prebuilt version of matplotplusplus for WebRTC and websocket connections. Made by the Mars Rover Design Team."
    } > /tmp/pkg/matplotplusplus_${MATPLOT_VERSION}_amd64/DEBIAN/control

    # Download LibDataChannel
    git clone --recurse-submodules --depth 1 --branch ${MATPLOT_VERSION} https://github.com/alandefreitas/matplotplusplus.git matplotplusplus
    mkdir matplotplusplus/build
    cd matplotplusplus/build

    # Build LibDataChannel
    cmake \
    -D CMAKE_INSTALL_PREFIX=/tmp/pkg/matplotplusplus_${MATPLOT_VERSION}_amd64/usr/local \
    -D CMAKE_BUILD_TYPE=Release ..

    # Install LibDataChannel
    make
    make install

    # Cleanup Install
    cd ../..
    rm -rf matplotplusplus

    # Create Package
    dpkg --build /tmp/pkg/matplotplusplus_${MATPLOT_VERSION}_amd64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/matplotplusplus_${MATPLOT_VERSION}_amd64.deb /tmp/pkg/deb/matplotplusplus_${MATPLOT_VERSION}_amd64.deb
fi
