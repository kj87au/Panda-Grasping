# This file is the build environment to use GIGA-Net

# Use the Nvidia as a base image
FROM nvidia/cuda:11.0.3-devel-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Australia/Brisbane
ENV CONDA_DIR /opt/conda

# Install base utilities
RUN apt update && \
    apt-get install --no-install-recommends -y build-essential apt-utils vim software-properties-common  && \
    apt-get install -y wget ffmpeg libsm6 libxext6 freeglut3-dev && \
    add-apt-repository -y ppa:deadsnakes/ppa && \
    apt install --no-install-recommends -y python3.7 python3-pip python3-setuptools python3-distutils && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda

# Put conda in path so we can use conda activate
ENV PATH=$CONDA_DIR/bin:$PATH

RUN conda init

CMD cd panda_grasp/nets/GIGA-main
