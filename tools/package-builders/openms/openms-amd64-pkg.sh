#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
OPENMS_VERSION="3.4.1"

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
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/openms/amd64/openms_${OPENMS_VERSION}_amd64.deb"

# Download the latest version
if [[ "$DOWNLOAD_LATEST" == true ]]; then
    echo "Downloading the latest version..."
    
    # Cleanup the download directory
    rm -rf /tmp/pkg
    rm -rf /tmp/OpenMS
    mkdir -p /tmp/pkg/deb

    # Download the package from the repository
    curl -L $FILE_URL --output /tmp/pkg/deb/openms_${OPENMS_VERSION}_amd64.deb

    # Exit the script
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
    exit 0
fi

# Check if the file exists
if [[ "$FORCE_BUILD" == false ]] && curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${OPENMS_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
    exit 0
else
    if [[ "$CHECK_PACKAGE" == true ]]; then
        echo "Package version ${FFMPEG_VERSION} does not exist in the repository. We're in check mode, so we're exiting with status 1."
        echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
        exit 1
    else
        echo "Package version ${OPENMS_VERSION} does not exist in the repository. Building the package."
        echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
        
        # Delete Old Packages
        rm -rf /tmp/pkg
        rm -rf /tmp/OpenMS

        # Create Package Directory
        mkdir -p /tmp/pkg/openms_${OPENMS_VERSION}_amd64/usr/local
        mkdir -p /tmp/pkg/openms_${OPENMS_VERSION}_amd64/DEBIAN

        # Create Control File
        {
            echo "Package: openms-mrdt"
            echo "Version: ${OPENMS_VERSION}"
            echo "Maintainer: OpenMS"
            echo "Depends:"
            echo "Architecture: amd64"
            echo "Homepage: https://github.com/OpenMS/OpenMS.git"
            echo "Description: A prebuilt version of OpenMS. Made by the Mars Rover Design Team."
        } > /tmp/pkg/openms_${OPENMS_VERSION}_amd64/DEBIAN/control

        # Apt install some stuff needed for building.
        sudo apt update
        sudo apt install -y libtool ninja-build

        # Download OpenMS
        git clone --depth 1 --branch release/${OPENMS_VERSION} https://github.com/OpenMS/OpenMS.git

        # Prepare contrib build. (all deps from source)
        cd OpenMS
        git submodule update --init contrib

        mkdir contrib-build
        cd contrib-build
        cmake -DBUILD_TYPE=ALL -DNUMBER_OF_JOBS=$(nproc) ../contrib
        make -j$(nproc)
        cd ..

        # Build OpenMS out-of-source.
        mkdir openms-build
        cd openms-build

        cmake -G Ninja \
            -DCMAKE_INSTALL_PREFIX=/tmp/pkg/openms_${OPENMS_VERSION}_amd64/usr/local \
            -DCMAKE_BUILD_TYPE=Release \
            -DOPENMS_CONTRIB_LIBS="$(realpath ../contrib-build)" \
            -DBOOST_USE_STATIC=ON \
            -DWITH_GUI=OFF \
            -DHAS_XSERVER=OFF \
            -DENABLE_DOCS=OFF \
            ..

        ninja -j$(nproc)
        ninja install

        # Cleanup sources after install
        cd ..
        rm -rf OpenMS contrib-build openms-build

        # Create Package
        dpkg --build /tmp/pkg/openms_${OPENMS_VERSION}_amd64

        # Create Package Directory
        mkdir -p /tmp/pkg/deb

        # Copy Package
        cp /tmp/pkg/openms_${OPENMS_VERSION}_amd64.deb /tmp/pkg/deb/openms_${OPENMS_VERSION}_amd64.deb
    fi
fi
