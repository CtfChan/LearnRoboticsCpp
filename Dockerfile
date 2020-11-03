FROM gcc:8 AS deps_stage

LABEL maintainer="Chris Chan"

# install cmake
RUN wget https://github.com/Kitware/CMake/releases/download/v3.18.2/cmake-3.18.2-Linux-x86_64.sh \
      -q -O /tmp/cmake-install.sh \
      && chmod u+x /tmp/cmake-install.sh \
      && mkdir /usr/bin/cmake \
      && /tmp/cmake-install.sh --skip-license --prefix=/usr/bin/cmake \
      && rm /tmp/cmake-install.sh
ENV PATH="/usr/bin/cmake/bin:${PATH}"

# deps
RUN apt-get update &&\
    apt-get install -y --no-install-recommends \
    libboost-all-dev libopencv-dev python3-opencv libeigen3-dev cppad gnuplot &&\
    apt-get clean

# ipopt
COPY install_ipopt.sh /
RUN  wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip &&\
     unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip
RUN  bash install_ipopt.sh /Ipopt-3.12.7

# ceres
RUN apt-get install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev
RUN wget http://ceres-solver.org/ceres-solver-2.0.0.tar.gz
RUN tar zxf ceres-solver-2.0.0.tar.gz
RUN mkdir ceres-bin
RUN cd ceres-bin && cmake ../ceres-solver-2.0.0 && make -j3 && make install

FROM deps_stage AS build_stage

COPY CMakeLists.txt /root/LearnRoboticsCpp/CMakeLists.txt
COPY src /root/LearnRoboticsCpp/src
COPY include /root/LearnRoboticsCpp/include
COPY examples /root/LearnRoboticsCpp/examples

WORKDIR /root/LearnRoboticsCpp
RUN mkdir build && cd build && cmake .. && make -j 4


