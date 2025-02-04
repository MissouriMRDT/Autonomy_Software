#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
GTEST_VERSION="1.15.2"
FORCE_BUILD=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --force|-f)
            FORCE_BUILD=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/gtest/arm64/gtest_${GTEST_VERSION}_arm64.deb"

# Check if the file exists
if [[ "$FORCE_BUILD" == false ]] && curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${GTEST_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
else
    echo "Package version ${GTEST_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
    
    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/googletest

    # Create Package Directory
    mkdir -p /tmp/pkg/gtest_${GTEST_VERSION}_arm64/usr/local
    mkdir -p /tmp/pkg/gtest_${GTEST_VERSION}_arm64/DEBIAN

    # Create Control File
    {
        echo "Package: googletest-mrdt"
        echo "Version: ${GTEST_VERSION}"
        echo "Maintainer: Google"
        echo "Depends:"
        echo "Architecture: arm64"
        echo "Homepage: https://google.github.io/googletest/"
        echo "Description: A prebuilt version of Google Test. Made by the Mars Rover Design Team."
    } > /tmp/pkg/gtest_${GTEST_VERSION}_arm64/DEBIAN/control

    # Download Google Test
    git clone --depth 1 --branch v${GTEST_VERSION} https://github.com/google/googletest.git
    mkdir googletest/build
    cd googletest/build

    # Build Google Test
    cmake \
    -D CMAKE_INSTALL_PREFIX=/tmp/pkg/gtest_${GTEST_VERSION}_arm64/usr/local ..

    # Install Google Test
    make
    make install

    # Cleanup Install
    cd ../..
    rm -rf googletest

    # Create Package
    dpkg --build /tmp/pkg/gtest_${GTEST_VERSION}_arm64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/gtest_${GTEST_VERSION}_arm64.deb /tmp/pkg/deb/gtest_${GTEST_VERSION}_arm64.deb
fi
