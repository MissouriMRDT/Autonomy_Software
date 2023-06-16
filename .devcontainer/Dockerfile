# Base image
FROM nvidia/cuda:11.6.2-base-ubuntu20.04
ARG DEBIAN_FRONTEND=noninteractive

# Install necessary packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    python3 \
    libboost-dev

# Optionally install the cmake for vcpkg
ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="3.22.2"
COPY ./reinstall-cmake.sh /tmp/

RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
    chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
    fi \
    && rm -f /tmp/reinstall-cmake.sh

# Clone your C++ project repository
RUN git clone https://github.com/Byrdman32/Autonomy_Software.git

# Set the working directory TO BUILD
WORKDIR Autonomy_Software/
RUN mkdir build; \
    cd build

# # Create and move into build dir. Autodetect GPU compatibility, and run cmake.
# RUN mkdir build; \
#     cd build; \
#     cmake -DGPU_SUPPORT=$(python -c "import torch; print(int(torch.cuda.is_available()))") .

# # Build the project
# RUN make

# [Optional] Uncomment this section to install additional vcpkg ports.
# RUN su vscode -c "${VCPKG_ROOT}/vcpkg install <your-port-name-here>"

# [Optional] Uncomment this section to install additional packages.
# RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
#     && apt-get -y install --no-install-recommends <your-package-list-here>
