#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
GEOLIB_VERSION="2.5"

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
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/geolib/amd64/geolib_${GEOLIB_VERSION}_amd64.deb"

# Download the latest version
if [[ "$DOWNLOAD_LATEST" == true ]]; then
    echo "Downloading the latest version..."
    
    # Cleanup the download directory
    rm -rf /tmp/pkg
    rm -rf /tmp/geolib
    mkdir -p /tmp/pkg/deb

    # Download the package from the repository
    curl -L $FILE_URL --output /tmp/pkg/deb/geolib_${GEOLIB_VERSION}_amd64.deb

    # Exit the script
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
    exit 0
fi

# Check if the file exists
if [[ "$FORCE_BUILD" == false ]] && curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${GEOLIB_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
    exit 0
else
    if [[ "$CHECK_PACKAGE" == true ]]; then
        echo "Package version ${FFMPEG_VERSION} does not exist in the repository. We're in check mode, so we're exiting with status 1."
        echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
        exit 1
    else
        echo "Package version ${GEOLIB_VERSION} does not exist in the repository. Building the package."
        echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
        
        # Delete Old Packages
        rm -rf /tmp/pkg
        rm -rf /tmp/geographiclib

        # Create Package Directory
        mkdir -p /tmp/pkg/geolib_${GEOLIB_VERSION}_amd64/usr/local
        mkdir -p /tmp/pkg/geolib_${GEOLIB_VERSION}_amd64/DEBIAN

        # Create Control File
        {
            echo "Package: geographiclib-mrdt"
            echo "Version: ${GEOLIB_VERSION}"
            echo "Maintainer: GeographicLib"
            echo "Depends:"
            echo "Architecture: amd64"
            echo "Homepage: https://geographiclib.sourceforge.io/"
            echo "Description: A prebuilt version of GeographicLib. Made by the Mars Rover Design Team."
        } > /tmp/pkg/geolib_${GEOLIB_VERSION}_amd64/DEBIAN/control

        # Download GeographicLib
        git clone --depth 1 --branch v${GEOLIB_VERSION} https://github.com/geographiclib/geographiclib.git
        mkdir geographiclib/build
        cd geographiclib/build

        # Build GeographicLib
        cmake \
        -D CMAKE_INSTALL_PREFIX=/tmp/pkg/geolib_${GEOLIB_VERSION}_amd64/usr/local \
        -D CMAKE_BUILD_TYPE=Release ..

        # Install GeographicLib
        make
        make install

        # Cleanup Install
        cd ../..
        rm -rf geographiclib

        # Create Package
        dpkg --build /tmp/pkg/geolib_${GEOLIB_VERSION}_amd64

        # Create Package Directory
        mkdir -p /tmp/pkg/deb

        # Copy Package
        cp /tmp/pkg/geolib_${GEOLIB_VERSION}_amd64.deb /tmp/pkg/deb/geolib_${GEOLIB_VERSION}_amd64.deb
    fi
fi
