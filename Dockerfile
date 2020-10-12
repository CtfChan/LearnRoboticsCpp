FROM ubuntu:18.04

LABEL maintainer="Chris Chan <christophertzechan@gmail.com>"

# Install necessary dependencies
RUN apt-get update &&\
    apt-get install -y --no-install-recommends \
        gnuplot \
        libboost-all-dev \
        libopencv-dev \
        libeigen3-dev \
        build-essential \
        autoconf \
        automake \
        libtool \
        pkg-config \
        apt-transport-https \
        ca-certificates \
        software-properties-common \
        wget \
        git \
        curl \
        gnupg \
        zlib1g-dev \
        swig \
        vim \
        gdb \
        valgrind \
        locales \
        locales-all &&\
    apt-get clean

# Install CMake
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | apt-key add - &&\
    apt-add-repository "deb https://apt.kitware.com/ubuntu/ bionic main" &&\
    apt-get update &&\
    apt-get install -y cmake

ENV LC_ALL en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8