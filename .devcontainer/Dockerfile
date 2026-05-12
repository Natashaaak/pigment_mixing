# Use Ubuntu 24.04 as the base image
FROM ubuntu:24.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install required dependencies
RUN apt-get update && apt-get install -y \
    cmake \
    git \
    build-essential \
    ninja-build \
    curl \
    zip \
    unzip \
    tar \
    pkg-config \
    # Matter dependencies:
    libeigen3-dev \
    libopenvdb-dev \
    libtbb-dev \
    libboost-all-dev \
    libilmbase-dev \
    libopenexr-dev \
    # Raytracing (OpenGL/X11) dependencies:
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libx11-dev \
    libxcursor-dev \
    libxinerama-dev \
    libxrandr-dev \
    libxi-dev \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Install vcpkg into /opt
RUN git clone https://github.com/microsoft/vcpkg.git /opt/vcpkg && \
    /opt/vcpkg/bootstrap-vcpkg.sh
	
# set variables
ENV VCPKG_ROOT=/opt/vcpkg
ENV PATH="${PATH}:${VCPKG_ROOT}"

# Set working directory
WORKDIR /workspace

# Copy vcpkg dependencies
COPY vcpkg.json /workspace/vcpkg.json

# Install libraries
RUN vcpkg install --x-install-root=/workspace/vcpkg_installed

# Copy project 
COPY . /workspace

# Set CMake toolchainu
ENV CMAKE_TOOLCHAIN_FILE=/opt/vcpkg/scripts/buildsystems/vcpkg.cmake
