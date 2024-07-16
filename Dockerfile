# Dockerfile for OpenFace setup based on Unix installation instructions
FROM eng_4_base

LABEL maintainer="Francesco Vigni <vignif@gmail.com>"

# Set environment variables
ENV DISABLE_BCOLZ_AVX2=true
ENV DEBIAN_FRONTEND=noninteractive 
    
RUN apt-get update 

RUN pip uninstall -y opencv-python \
    pip uninstall -y opencv-contrib-python \
    pip uninstall -y opencv-contrib-python-headless \
    pip install opencv-contrib-python==4.5.5.62

# Set environment variables
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}
ENV CUDA_VISIBLE_DEVICES=0 


RUN /entrypoint.sh
CMD [ "/bin/bash" ]
