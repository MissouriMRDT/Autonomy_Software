#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
QT6_VERSION="6.5.0"

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

# Define Package URL (adjust the URL to your hosting location)
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/qt6/amd64/qt6_${QT6_VERSION}_amd64.deb"

# Download the latest version
if [[ "$DOWNLOAD_LATEST" == true ]]; then
    echo "Downloading the latest version..."
    
    # Cleanup the download directory
    rm -rf /tmp/pkg
    rm -rf /tmp/qt6
    rm -rf /tmp/qtshadertools
    rm -rf /tmp/qt6_quick
    rm -rf /tmp/pkg/deb
    mkdir -p /tmp/pkg/deb

    # Download the package from the repository
    curl -L $FILE_URL --output /tmp/pkg/deb/qt6_${QT6_VERSION}_amd64.deb

    # Exit the script
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
    exit 0
fi

# Check if the file exists
if [[ "$FORCE_BUILD" == false ]] && curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${QT6_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
    exit 0
else
    if [[ "$CHECK_PACKAGE" == true ]]; then
        echo "Package version ${QT6_VERSION} does not exist in the repository. We're in check mode, so exiting with status 1."
        echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
        exit 1
    else
        echo "Package version ${QT6_VERSION} does not exist in the repository. Building the package."
        echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
        
        # Delete Old Packages
        rm -rf /tmp/pkg
        rm -rf /tmp/qt6

        # Create Package Directory
        mkdir -p /tmp/pkg/qt6_${QT6_VERSION}_amd64/usr/local
        mkdir -p /tmp/pkg/qt6_${QT6_VERSION}_amd64/DEBIAN

        # Create Control File
        {
            echo "Package: qt6-mrdt"
            echo "Version: ${QT6_VERSION}"
            echo "Maintainer: Qt Project"
            echo "Depends:"
            echo "Architecture: amd64"
            echo "Homepage: https://www.qt.io/"
            echo "Description: A built version of Qt6 Base Module with Qt Quick."
        } > /tmp/pkg/qt6_${QT6_VERSION}_amd64/DEBIAN/control

        # Install prerequisites for building qt6
        sudo apt update
        sudo apt install -y build-essential cmake ninja-build git

        # Clone qtbase repository from Qt Project
        git clone --depth 1 --branch v${QT6_VERSION} https://code.qt.io/qt/qtbase.git qt6
        cd qt6

        # Create build directory for qt6 base
        mkdir build && cd build

        # Configure via cmake (using Ninja)
        cmake -G Ninja \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_INSTALL_PREFIX=/tmp/pkg/qt6_${QT6_VERSION}_amd64/usr/local \
            ..

        # Build and install qt6 base
        ninja
        ninja install

        # Cleanup sources after install of qt6 base
        cd /tmp
        rm -rf qt6

        # Build qt6 shadertools (from the qtshadertools repository)
        git clone --depth 1 --branch v${QT6_VERSION} https://code.qt.io/qt/qtshadertools.git
        cd qtshadertools
        mkdir build && cd build

        # Configure via cmake (using Ninja)
        cmake -G Ninja \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_INSTALL_PREFIX=/tmp/pkg/qt6_${QT6_VERSION}_amd64/usr/local \
            -DCMAKE_PREFIX_PATH=/tmp/pkg/qt6_${QT6_VERSION}_amd64/usr/local \
            ..

        # Build and install qt6 shadertools.
        ninja
        ninja install

        # Cleanup sources after install of qt6 shadertools
        cd /tmp
        rm -rf qtshadertools

        # Build qt6 quick (from the qtdeclarative repository)
        git clone --depth 1 --branch v${QT6_VERSION} https://code.qt.io/qt/qtdeclarative.git qt6_quick
        cd qt6_quick

        # Create build directory for qt6 quick
        mkdir build && cd build

        # Configure via cmake (using Ninja)
        cmake -G Ninja \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_INSTALL_PREFIX=/tmp/pkg/qt6_${QT6_VERSION}_amd64/usr/local \
            -DCMAKE_PREFIX_PATH=/tmp/pkg/qt6_${QT6_VERSION}_amd64/usr/local \
            ..

        # Build and install qt6 quick.
        ninja
        ninja install

        # Cleanup sources after install of qt6 quick
        cd /tmp
        rm -rf qt6_quick

        # Build deb package
        dpkg --build /tmp/pkg/qt6_${QT6_VERSION}_amd64

        # Create Package Directory if it doesn't exist
        mkdir -p /tmp/pkg/deb

        # Copy Package
        cp /tmp/pkg/qt6_${QT6_VERSION}_amd64.deb /tmp/pkg/deb/qt6_${QT6_VERSION}_amd64.deb
    fi
fi
