#!/bin/bash
set -euo pipefail

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
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/openms/arm64/openms_${OPENMS_VERSION}_arm64.deb"

# Helper: safely write GitHub Actions outputs if available, otherwise echo
gh_out() {
    if [[ -n "${GITHUB_OUTPUT-}" ]]; then
        echo "$1" >> "$GITHUB_OUTPUT"
    else
        echo "$1"
    fi
}

# ------------------------------------------------------------------
# Download Handling
# ------------------------------------------------------------------
if [[ "$DOWNLOAD_LATEST" == true ]]; then
    echo "Downloading the latest version..."
    rm -rf /tmp/pkg
    mkdir -p /tmp/pkg/deb
    curl -L --fail --show-error --output "/tmp/pkg/deb/openms_${OPENMS_VERSION}_arm64.deb" "$FILE_URL"
    gh_out "rebuilding_pkg=false"
    exit 0
fi

if [[ "$FORCE_BUILD" == false ]] && curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${OPENMS_VERSION} already exists. Skipping build."
    gh_out "rebuilding_pkg=false"
    exit 0
else
    if [[ "$CHECK_PACKAGE" == true ]]; then
        echo "Package version ${OPENMS_VERSION} does not exist. Exiting check mode."
        gh_out "rebuilding_pkg=true"
        exit 1
    else
        echo "Package version ${OPENMS_VERSION} does not exist. Starting build process."
        gh_out "rebuilding_pkg=true"

        # ------------------------------------------------------------------
        # Build Preparation
        # ------------------------------------------------------------------
        
        # Clean previous builds
        rm -rf /tmp/pkg
        rm -rf /tmp/openms_src
        rm -rf /tmp/contrib_build
        rm -rf /tmp/openms_build

        # Structure setup
        PKG_DIR="/tmp/pkg/openms_${OPENMS_VERSION}_arm64"
        mkdir -p "${PKG_DIR}/usr/local"
        mkdir -p "${PKG_DIR}/DEBIAN"

        # Control File
        {
            echo "Package: openms-mrdt"
            echo "Version: ${OPENMS_VERSION}"
            echo "Maintainer: Missouri MRDT"
            echo "Architecture: arm64"
            echo "Homepage: https://github.com/OpenMS/OpenMS.git"
            echo "Description: A prebuilt version of OpenMS ${OPENMS_VERSION} for ARM64."
        } > "${PKG_DIR}/DEBIAN/control"

        # ------------------------------------------------------------------
        # Dependency Installation
        # ------------------------------------------------------------------
        sudo apt update
        # Installs all necessary dev libraries so we don't need to build them in contrib
        sudo apt install -y \
          autoconf patch libtool git \
          libeigen3-dev libboost-all-dev libxerces-c-dev \
          zlib1g-dev libsvm-dev libbz2-dev coinor-libcoinmp-dev libhdf5-dev \
          libglpk-dev ninja-build dpkg-dev

        # ------------------------------------------------------------------
        # Source Retrieval
        # ------------------------------------------------------------------
        echo "Cloning OpenMS..."
        if git clone --depth 1 --branch "release/${OPENMS_VERSION}" "https://github.com/OpenMS/OpenMS.git" /tmp/openms_src; then
            echo "Cloned release branch."
        elif git clone --depth 1 --branch "${OPENMS_VERSION}" "https://github.com/OpenMS/OpenMS.git" /tmp/openms_src; then
            echo "Cloned tag."
        else
            echo "Release/Tag not found, cloning default branch."
            git clone --depth 1 "https://github.com/OpenMS/OpenMS.git" /tmp/openms_src
        fi

        # ------------------------------------------------------------------
        # OpenMS Build
        # ------------------------------------------------------------------
        # NOTE: We skip building 'contrib' because we installed all dependencies
        # (libsvm, coinmp, eigen, boost, etc.) via apt.
        
        mkdir -p /tmp/openms_build
        cd /tmp/openms_build

        echo "Configuring OpenMS..."
        
        # We explicitly set Search Engines to common system paths or empty if unused
        # We REMOVE -DOPENMS_CONTRIB_LIBS to force it to look in system paths (/usr/lib)
        
        cmake -G Ninja \
            -DCMAKE_INSTALL_PREFIX="${PKG_DIR}/usr/local" \
            -DCMAKE_BUILD_TYPE=Release \
            -DBOOST_USE_STATIC=OFF \
            -DWITH_GUI=OFF \
            -DHAS_XSERVER=OFF \
            -DENABLE_DOCS=OFF \
            -DCMAKE_PREFIX_PATH="/usr/lib/aarch64-linux-gnu/cmake;/usr/lib/cmake;/usr/local" \
            /tmp/openms_src

        echo "Building OpenMS..."
        ninja -j"$(nproc)" || { echo "OpenMS Build failed"; exit 1; }
        
        echo "Installing OpenMS to packaging directory..."
        ninja install || { echo "Install step failed"; exit 1; }

        # ------------------------------------------------------------------
        # Packaging
        # ------------------------------------------------------------------
        echo "Building .deb package..."
        dpkg --build "${PKG_DIR}" || { echo "dpkg build failed"; exit 1; }

        # Move to export location
        mkdir -p /tmp/pkg/deb
        cp "/tmp/pkg/openms_${OPENMS_VERSION}_arm64.deb" "/tmp/pkg/deb/openms_${OPENMS_VERSION}_arm64.deb"

        # Cleanup Source/Build dirs
        rm -rf /tmp/openms_src /tmp/openms_build

        gh_out "rebuilding_pkg=false"
        echo "Build complete: /tmp/pkg/deb/openms_${OPENMS_VERSION}_arm64.deb"
    fi
fi