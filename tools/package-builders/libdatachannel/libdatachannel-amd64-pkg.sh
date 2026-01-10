#!/bin/bash
set -euo pipefail

# Set Working Directory
cd /tmp

# Install Variables
LIBDATACHANNEL_VERSION="0.23.2"

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
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/libdatachannel/amd64/libdatachannel_${LIBDATACHANNEL_VERSION}_amd64.deb"

# Helper: safely write GitHub Actions outputs if available, otherwise echo
gh_out() {
    if [[ -n "${GITHUB_OUTPUT-}" ]]; then
        echo "$1" >> "$GITHUB_OUTPUT"
    else
        echo "$1"
    fi
}

# Download the latest version
if [[ "$DOWNLOAD_LATEST" == true ]]; then
    echo "Downloading the latest version..."
    
    # Cleanup the download directory
    rm -rf /tmp/pkg
    rm -rf /tmp/libdatachannel
    mkdir -p /tmp/pkg/deb

    # Download the package from the repository
    curl -L $FILE_URL --output /tmp/pkg/deb/libdatachannel_${LIBDATACHANNEL_VERSION}_amd64.deb

    # Exit the script
    gh_out "rebuilding_pkg=false"
    exit 0
fi

# Check if the file exists
if [[ "$FORCE_BUILD" == false ]] && curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${LIBDATACHANNEL_VERSION} already exists in the repository. Skipping build."
    gh_out "rebuilding_pkg=false"
    exit 0
else
    if [[ "$CHECK_PACKAGE" == true ]]; then
        echo "Package version ${FFMPEG_VERSION} does not exist in the repository. We're in check mode, so we're exiting with status 1."
        gh_out "rebuilding_pkg=true"
        exit 1
    else
        echo "Package version ${LIBDATACHANNEL_VERSION} does not exist in the repository. Building the package."
        gh_out "rebuilding_pkg=true"
        
        # Delete Old Packages
        rm -rf /tmp/pkg
        rm -rf /tmp/libdatachannel

        # Create Package Directory
        mkdir -p /tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_amd64/usr/local
        mkdir -p /tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_amd64/DEBIAN

        # Create Control File
        {
            echo "Package: libdatachannel-mrdt"
            echo "Version: ${LIBDATACHANNEL_VERSION}"
            echo "Maintainer: paullouisageneau"
            echo "Depends:"
            echo "Architecture: amd64"
            echo "Homepage: https://github.com/paullouisageneau/libdatachannel"
            echo "Description: A prebuilt version of libdatachannel for WebRTC and websocket connections. Made by the Mars Rover Design Team."
        } > /tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_amd64/DEBIAN/control

        # Download LibDataChannel
        git clone --recurse-submodules --depth 1 --branch v${LIBDATACHANNEL_VERSION} https://github.com/paullouisageneau/libdatachannel.git libdatachannel
        mkdir libdatachannel/build
        cd libdatachannel/build

        # Build LibDataChannel
        cmake \
        -D CMAKE_INSTALL_PREFIX=/tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_amd64/usr/local \
        -D USE_GNUTLS=0 -D USE_NICE=0 \
        -D BUILD_SHARED_LIBS=OFF \
        -D CMAKE_BUILD_TYPE=Release ..

        # Install LibDataChannel
        make datachannel-static
        make install datachannel-static

        # Cleanup Install
        cd ../..
        rm -rf libdatachannel

        # Create Package
        dpkg --build /tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_amd64

        # Create Package Directory
        mkdir -p /tmp/pkg/deb

        # Copy Package
        cp /tmp/pkg/libdatachannel_${LIBDATACHANNEL_VERSION}_amd64.deb /tmp/pkg/deb/libdatachannel_${LIBDATACHANNEL_VERSION}_amd64.deb
    fi
fi
