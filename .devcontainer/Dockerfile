# Base Image
FROM stereolabs/zed:3.0-devel-jetson-jp4.2
ARG DEBIAN_FRONTEND=noninteractive

# Install Necessary Packages
RUN apt-get update || true && apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
	build-essential gcc g++ gdb \
	cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
	libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev \
    yasm libatlas-base-dev gfortran libpq-dev \
    libxine2-dev libglew-dev libtiff5-dev zlib1g-dev libavutil-dev libpostproc-dev \
    libeigen3-dev python3-dev python3-pip python3-numpy libx11-dev tzdata \
    && rm -rf /var/lib/apt/lists/*

# Set Linux Environment Variable for Make Threads. Runs everytime a terminal is opened.
RUN echo 'export MAKEFLAGS=-j$(($(grep -c "^processor" /proc/cpuinfo) - 1))' >> .bashrc

# Install CMake
ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="3.24.3"
COPY ./reinstall-cmake.sh /tmp/

RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
        chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
    fi && \
    rm -f /tmp/reinstall-cmake.sh

# Setup Environment
ENV DEBIAN_FRONTEND=noninteractive
ENV CUDA_HOME="/usr/local/cuda"
ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"

# Set Working Directory
WORKDIR /opt

# Install OpenCV from Source
ARG OPENCV_VERSION="4.7.0"

# Install Build Dependencies
COPY ./opencv_install_deps.sh /tmp/
RUN chmod +x /tmp/opencv_install_deps.sh && \
    /tmp/opencv_install_deps.sh &&  \
    rm -f /tmp/opencv_install_deps.sh

# OpenCV looks for the cuDNN version in cudnn_version.h, but it's been renamed to cudnn_version_v8.h
RUN ln -s /usr/include/$(uname -i)-linux-gnu/cudnn_version_v8.h /usr/include/$(uname -i)-linux-gnu/cudnn_version.h

# Install OpenCV from Source
RUN git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv.git && \
    git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git && \
    cd opencv && \
    mkdir build && \
    cd build && \
    cmake \
	-D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/ \
	-D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
	-D WITH_V4L=ON \
	-D WITH_QT=OFF \
	-D WITH_OPENGL=ON \
	-D WITH_GSTREAMER=ON \
	-D OPENCV_GENERATE_PKGCONFIG=ON \
	-D OPENCV_ENABLE_NONFREE=ON \
	-D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules \
	-D INSTALL_PYTHON_EXAMPLES=OFF \
	-D INSTALL_C_EXAMPLES=OFF \
	-D BUILD_EXAMPLES=OFF .. && \
    make -j"$(nproc)" && \
    make install

# Clone your C++ project repository
RUN git clone https://github.com/Byrdman32/Autonomy_Software.git

# Set the working directory TO BUILD
WORKDIR Autonomy_Software/
RUN mkdir build; \
    cd build
