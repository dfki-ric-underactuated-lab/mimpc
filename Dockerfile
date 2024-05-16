FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive

# Install additional ros packages and other libraries
RUN apt-get -q update && apt-get -qy install  \
    vim \
    git \
    make \
    wget \
    xterm \
    iputils-ping \
    iproute2 \
    curl\
    g++\
    cmake

######### Mandatory depdendencies
# Install SCIP solver
RUN apt-get -qy install wget cmake g++ m4 xz-utils libgmp-dev unzip zlib1g-dev libboost-program-options-dev libboost-serialization-dev libboost-regex-dev libboost-iostreams-dev libtbb-dev libreadline-dev pkg-config git liblapack-dev libgsl-dev flex bison libcliquer-dev gfortran file dpkg-dev libopenblas-dev rpm libtbb2
RUN cd /opt && wget https://www.scipopt.org/download/release/SCIPOptSuite-8.0.3-Linux-ubuntu.sh && yes | sh SCIPOptSuite-8.0.3-Linux-ubuntu.sh
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/SCIPOptSuite-8.0.3-Linux/lib"
ENV PATH="${PATH}:/opt/SCIPOptSuite-8.0.3-Linux/bin"
RUN rm /opt/SCIPOptSuite-8.0.3-Linux-ubuntu.sh
# Install EigenLib
RUN apt-get -qy install libeigen3-dev
# Install fmt
WORKDIR /usr/local/src
RUN git clone https://github.com/fmtlib/fmt && cd fmt && git checkout 8.1.1
RUN cd fmt && mkdir build && cd build && cmake .. && make install
########## Dependencies for simulation and examples
# Install drake
RUN wget https://github.com/RobotLocomotion/drake/releases/download/v1.25.0/drake-dev_1.25.0-1_amd64-jammy.deb && apt-get -qy install --no-install-recommends ./drake-dev_1.25.0-1_amd64-jammy.deb
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/drake/lib"
ENV PATH="${PATH}:/opt/drake/bin"
RUN rm drake-dev_1.25.0-1_amd64-jammy.deb
# Intsall ZLIB
RUN apt-get -qy install zlib1g-dev
# Install Cnpy
WORKDIR /usr/local/src
RUN git clone https://github.com/rogersce/cnpy
RUN cd cnpy && mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. && make install
######### Install MIMPC
RUN apt-get -qy install  doxygen
COPY examples /home/mimpc/examples
COPY include /home/mimpc/include
COPY models /home/mimpc/models
COPY src /home/mimpc/src
COPY CMakeLists.txt /home/mimpc/
COPY Doxyfile /home/mimpc/
COPY INSTALL.md /home/mimpc/
COPY README.md /home/mimpc/

WORKDIR /home/mimpc
RUN mkdir build && cd build &&  cmake .. -DBuildSim=ON -DBuildExamples=ON && make install
RUN mkdir docs && doxygen Doxyfile

CMD bash
