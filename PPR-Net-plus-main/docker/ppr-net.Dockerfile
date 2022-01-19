# This file is the build environment to use PPR-Net

FROM pytorch/pytorch:1.1.0-cuda10.0-cudnn7.5-devel

CMD pip3 install torchvision==0.3.0 opencv-python sklearn h5py nibabel
