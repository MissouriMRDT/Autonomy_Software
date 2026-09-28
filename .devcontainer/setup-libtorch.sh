#!/usr/bin/env bash
#-------------------------------------------------------------------------------------------------------------
# Checks for NVIDIA Blackwell (compute capability >= 12.0 / RTX 50-series) architecture and ensures
# compatible LibTorch (CUDA 12.8 / sm_120) is installed to /opt/libtorch.
#-------------------------------------------------------------------------------------------------------------
set -e

FORCE=0
if [[ "$1" == "--force" ]]; then
    FORCE=1
fi

# Check if /opt/libtorch is already fully installed
if [ -f "/opt/libtorch/lib/libtorch_cuda.so" ]; then
    # Ensure ldconfig is registered
    if [ ! -f "/etc/ld.so.conf.d/opt-libtorch.conf" ]; then
        echo "/opt/libtorch/lib" | sudo tee /etc/ld.so.conf.d/opt-libtorch.conf > /dev/null
        sudo ldconfig
    fi
    exit 0
fi

# Query GPU compute capability
IS_BLACKWELL=0
if command -v nvidia-smi &> /dev/null; then
    COMPUTE_CAP=$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -n 1 | cut -d"." -f1 || true)
    GPU_NAME=$(nvidia-smi --query-gpu=gpu_name --format=csv,noheader 2>/dev/null | head -n 1 || true)
    if [ -n "$COMPUTE_CAP" ] && [ "$COMPUTE_CAP" -ge 12 ] 2>/dev/null; then
        IS_BLACKWELL=1
    fi
fi

if [ "$FORCE" -eq 1 ] || [ "$IS_BLACKWELL" -eq 1 ]; then
    echo "=========================================================================="
    echo "NVIDIA Blackwell GPU detected (${GPU_NAME:-sm_120})."
    echo "Installing LibTorch with CUDA 12.8 (sm_120) support to /opt/libtorch..."
    echo "=========================================================================="

    TMP_DIR=$(mktemp -d -t libtorch-XXXXXXXXXX)
    cleanup() {
        rm -rf "${TMP_DIR}"
    }
    trap cleanup EXIT

    ZIP_URL="https://download.pytorch.org/libtorch/nightly/cu128/libtorch-cxx11-abi-shared-with-deps-latest.zip"
    echo "Downloading LibTorch package from ${ZIP_URL}..."
    curl -sSL -C - -o "${TMP_DIR}/libtorch.zip" "${ZIP_URL}"

    echo "Extracting LibTorch to /opt/libtorch..."
    sudo unzip -q -o "${TMP_DIR}/libtorch.zip" -d /opt/

    echo "/opt/libtorch/lib" | sudo tee /etc/ld.so.conf.d/opt-libtorch.conf > /dev/null
    sudo ldconfig

    echo "LibTorch successfully installed to /opt/libtorch."
    echo "=========================================================================="
fi
