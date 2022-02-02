# This file is the build environment to use PPR-Net

# Use the pytorch as a base image
FROM pytorch/pytorch:1.1.0-cuda10.0-cudnn7.5-devel

# Next we need to update and install
RUN apt update && apt upgrade -y
RUN apt install vim
RUN pip install torchvision==0.3.0 opencv-python sklearn h5py nibabel scipy

# Get into the correct directory on startup
CMD cd ../panda_grasp/
