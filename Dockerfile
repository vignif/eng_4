FROM opendr/opendr-toolkit:cuda_v3.0.0
LABEL maintainer="Francesco Vigni <vignif@gmail.com>"

# Install essential packages
RUN apt-get update 
ENV DISABLE_BCOLZ_AVX2=true

# Copy your source code to the container
COPY /catkin_ws /catkin_ws

WORKDIR /catkin_ws

# Install necessary packages
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    git \
    python3 \
    python3-pip \
    libfreetype6-dev \
    build-essential \
    cmake \
    python3-dev \
    wget \
    libopenblas-dev \
    libsndfile1 \
    libboost-dev \
    libeigen3-dev \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libavdevice-dev \
    libavfilter-dev \
    libavutil-dev \
    libswscale-dev \
    libavresample-dev \
    libopenmpi-dev \
    libswresample-dev && \
    rm -rf /var/lib/apt/lists/*


# Set Git access token as an environment variable
ENV GIT_ACCESS_TOKEN
RUN git clone https://${GIT_ACCESS_TOKEN}@github.com/vignif/grace_common_msgs.git /catkin_ws/src/grace_common_msgs


RUN pip install --upgrade pip setuptools wheel==0.38.4

RUN pip install catkin-pkg \
                catkin-pkg-modules \
                catkin-tools \ 
                wheel==0.38.4 \
                tsmoothie \ 
                networkx 

RUN pip install torch==1.13.1+cu116 torchvision==0.14.1+cu116 torchaudio==0.13.1 -f https://download.pytorch.org/whl/torch_stable.html
RUN pip install 'git+https://github.com/facebookresearch/detectron2.git'
RUN pip install mxnet-cu112==1.8.0post0 av
RUN pip install opendr-toolkit-engine
# RUN pip install opendr-toolkit-engine
RUN pip install opendr-toolkit-face-recognition
RUN pip install opendr-toolkit-pose-estimation

RUN git clone https://github.com/ros-perception/vision_opencv.git -b noetic ./src/vision_opencv

# RUN pip install opendr-toolkit

# COPY requirements.txt requirements.txt

# RUN pip install -r requirements.txt --user
# RUN pip install opendr-toolkit-engine

# Install pyhri from GitHub
RUN git clone https://github.com/ros4hri/pyhri.git ./src/pyhri

# Remove default ROS sources list to prevent conflicts
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list

# Initialize rosdep and install dependencies
RUN rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y

# Copy and set the entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
# RUN /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# CMD [ "/bin/bash" ]
