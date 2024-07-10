# Dockerfile for OpenFace setup based on Unix installation instructions
FROM ros:noetic

LABEL maintainer="Francesco Vigni <vignif@gmail.com>"

# Set environment variables
ENV DISABLE_BCOLZ_AVX2=true

# Install essential build tools and dependencies
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential \
    g++-8 \
    cmake \
    git \
    libopenblas-dev \
    libgtk2.0-dev \
    pkg-config \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    python-dev \
    python-numpy \
    python3-opencv \
    python3-rosdep \
    python3-catkin-tools \
    libtbb2 \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libdc1394-22-dev \
    unzip \
    wget \
    ros-noetic-catkin \
    ros-noetic-cv-bridge \
    libyaml-cpp-dev \
    ros-noetic-tf2* \
    apturl \
    python3-pip \
    libosmesa6-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libavdevice-dev \
    libavutil-dev \
    libavfilter-dev \
    libswresample-dev \
    python3.8-venv \
    libfreetype6-dev \
    libsndfile1 \
    libboost-dev \
    libeigen3-dev \
    ros-noetic-image-geometry \
    && rm -rf /var/lib/apt/lists/*

# Install GCC 8 if not already installed
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 90 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 90

# Install required version of CMake if not available
RUN cmake_version=$(cmake --version | grep -oP "(?<=cmake version )[\d\.]+") && \
    if dpkg --compare-versions "$cmake_version" "lt" "3.8"; then \
        mkdir -p /tmp/cmake_tmp && \
        cd /tmp/cmake_tmp && \
        wget https://cmake.org/files/v3.10/cmake-3.10.1.tar.gz && \
        tar -xzvf cmake-3.10.1.tar.gz -qq && \
        cd cmake-3.10.1/ && \
        ./bootstrap && \
        make -j$(nproc) && \
        make install && \
        cd / && \
        rm -rf /tmp/cmake_tmp; \
    fi

# Copy your source code to the container
COPY . /catkin_ws/src/engage/

WORKDIR /catkin_ws

# Update pip and install Python packages
RUN pip install --upgrade pip setuptools wheel && \
    pip install tsmoothie networkx wheel==0.38.4 && \
    pip install Cython && \ 
    pip install opendr-toolkit-engine av opendr-toolkit

    # Delete default sources list file if it exists
    
    
RUN git clone https://github.com/ros4hri/pyhri.git ./src/pyhri
    
RUN apt-get update 

RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list

# Initialize rosdep and install dependencies
RUN rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y

CMD [ "/bin/bash" ]

