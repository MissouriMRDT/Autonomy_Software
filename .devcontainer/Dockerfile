# Base Image
FROM stereolabs/zed:4.0-devel-jetson-jp5.1.1
ARG DEBIAN_FRONTEND=noninteractive

# Set Linux Environment Variable for Make Threads. Runs everytime a terminal is opened.
RUN echo 'export MAKEFLAGS=-j$(($(grep -c "^processor" /proc/cpuinfo) - 1))' >> .bashrc

# Install Necessary Packages
RUN apt-get update && \
	apt-get install --no-install-recommends -y \
	build-essential gcc g++ gdb git \
	libgtk2.0-dev pkg-config \
	libavcodec-dev libavformat-dev libswscale-dev \
	libboost-all-dev \
	&& rm -rf /var/lib/apt/lists/*

# Install CMake
ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="3.24.3"
COPY ./reinstall-cmake.sh /tmp/

RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
	chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
	fi && \
	rm -f /tmp/reinstall-cmake.sh

# Set Working Directory
WORKDIR /opt

# Install OpenCV from Source
ARG OPENCV_VERSION="4.7.0"

# Install OpenCV from Source
RUN git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv.git && \
	git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git && \
	cp -r opencv_contrib/modules/aruco/ opencv/modules/ && \
	cd opencv && \
	mkdir build && \
	cd build && \
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D BUILD_SHARED_LIBS=NO .. && \
	cat /proc/cpuinfo | grep "processor" | wc -l | xargs make -j && \
	make install && cd ../.. && \
	rm -rf opencv_contrib && rm -rf opencv

# Clone your C++ project repository
RUN git clone https://github.com/Byrdman32/Autonomy_Software.git

# Set the working directory TO BUILD
WORKDIR /Autonomy_Software/
