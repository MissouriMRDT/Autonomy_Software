#!/bin/bash
set -euo pipefail

# Set Working Directory
cd /tmp

# Install Variables
VTK_VERSION="9.5.1"

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
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/vtk/amd64/vtk_${VTK_VERSION}_amd64.deb"

# Download the latest version
if [[ "$DOWNLOAD_LATEST" == true ]]; then
    echo "Downloading the latest version..."
    
    # Cleanup the download directory
    rm -rf /tmp/pkg
    rm -rf /tmp/VTK
    mkdir -p /tmp/pkg/deb

    # Download the package from the repository
    curl -L $FILE_URL --output /tmp/pkg/deb/vtk_${VTK_VERSION}_amd64.deb

    # Exit the script
    gh_out "rebuilding_pkg=false"
    exit 0
fi

# Check if the file exists
if [[ "$FORCE_BUILD" == false ]] && curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${VTK_VERSION} already exists in the repository. Skipping build."
    gh_out "rebuilding_pkg=false"
    exit 0
else
    if [[ "$CHECK_PACKAGE" == true ]]; then
        echo "Package version ${FFMPEG_VERSION} does not exist in the repository. We're in check mode, so we're exiting with status 1."
        gh_out "rebuilding_pkg=true"
        exit 1
    else
        echo "Package version ${VTK_VERSION} does not exist in the repository. Building the package."
        gh_out "rebuilding_pkg=true"
        
        # Delete Old Packages
        rm -rf /tmp/pkg
        rm -rf /tmp/VTK

        # Create Package Directory
        mkdir -p /tmp/pkg/vtk_${VTK_VERSION}_amd64/usr/local
        mkdir -p /tmp/pkg/vtk_${VTK_VERSION}_amd64/DEBIAN

        # Create Control File
        {
            echo "Package: vtk-mrdt"
            echo "Version: ${VTK_VERSION}"
            echo "Maintainer: VTK"
            echo "Depends:"
            echo "Architecture: amd64"
            echo "Homepage: https://github.com/Kitware/VTK.git"
            echo "Description: A prebuilt version of VTK. Made by the Mars Rover Design Team."
        } > /tmp/pkg/vtk_${VTK_VERSION}_amd64/DEBIAN/control

        # Download VTK
        git clone --recurse-submodules --depth 1 --branch v${VTK_VERSION} https://github.com/Kitware/VTK.git

        # Build VTK
        cd VTK
        mkdir build && cd build
        cmake \
            -DCMAKE_INSTALL_PREFIX=/tmp/pkg/vtk_${VTK_VERSION}_amd64/usr/local \
            -DVTK_QT_VERSION=6 \
            -DVTK_GROUP_ENABLE_Qt=YES \
            -DQt6_DIR="/usr/local/lib/cmake/Qt6" \
            -DQt6Quick_DIR="/usr/local/lib/cmake/Qt6Quick" \
            -DCMAKE_PREFIX_PATH="/usr/local/" \
            -DCMAKE_FIND_DEBUG_MODE=TRUE \
            -DQT_DEBUG_FIND_PACKAGE=ON \
            ..
        make
        make install

        # Cleanup Build Files
        cd ../..
        rm -rf VTK

        # Create Package
        dpkg --build /tmp/pkg/vtk_${VTK_VERSION}_amd64

        # Create Package Directory
        mkdir -p /tmp/pkg/deb

        # Copy Package
        cp /tmp/pkg/vtk_${VTK_VERSION}_amd64.deb /tmp/pkg/deb/vtk_${VTK_VERSION}_amd64.deb
    fi
fi
