# This file is the build environment to use PPR-Net

# Use the pytorch as a base image
FROM nvidia/cuda:11.1.1-cudnn8-devel-ubuntu20.04 

# Next we need to update and install
RUN apt update && \
    apt install --no-install-recommends -y vim build-essential software-properties-common && \
    add-apt-repository -y ppa:deadsnakes/ppa && \
    apt install --no-install-recommends -y python3.7 python3-pip python3-setuptools python3-distutils && \
    apt clean && rm -rf /var/lib/apt/lists/*

COPY ../contact_graspnet_env.yml /contact_graspnet_env.yml

RUN python3.7 -m pip install --upgrade pip && \
    python3.7 -m pip install --no-cache-dir -r /req.txt


