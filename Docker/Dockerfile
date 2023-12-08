# syntax=docker/dockerfile:1

FROM carlasim/carla:0.9.14

USER root

# prerequisites
RUN apt-get update ; \
  apt-get install -y \
  build-essential \
  gcc \
  clang \
  bison \
  re2c \
  cmake \
  wget \
  curl \
  git \
  fontconfig \
  fonts-freefont-ttf \
  libjpeg-turbo8 \
  libjpeg-turbo8-dev \
  libtbb2 \
  eog \
  vim \
  # for compiling python from source
  zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev libssl-dev libreadline-dev libffi-dev \
  # since didn't install precompiled python binaries
  libbz2-dev liblzma-dev
# For python wrappers for sqlite
RUN apt-get install -y libsqlite3-dev

# Needed for enabling TK with python
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=US/NewYork
RUN apt-get install -y tk-dev

# Install Python 3.8.18
WORKDIR /home/carla/python-setup
RUN wget https://www.python.org/ftp/python/3.8.17/Python-3.8.17.tgz ; \
  tar -xvf Python-3.8.17.tgz ; \
  cd Python-3.8.17 ; \
  ./configure --enable-loadable-sqlite-extensions \
              --enable-optimizations \
              --with-tcltk-includes='-I/usr/include' \
              --with-tcltk-libs='-L/usr/lib' ; \
  # Number of cpu cores to use:
  make -j12 ; \
  # Alternative installation since Ubuntu's GUI depends on the default Python 2.7
  make altinstall

# Setup python environment
USER carla
WORKDIR /home/carla
RUN python3.8 -m venv .venv

# Build Clingo with Python support
RUN . .venv/bin/activate ; \
  pip install --upgrade cmake ; \
  # cffi needed for python support
  pip install cffi ; \ 
  git clone --depth 1 --branch v5.6.2 https://github.com/potassco/clingo.git ; \
  git -C /home/carla/clingo submodule update --init --recursive ; \
  . /home/carla/.venv/bin/activate ; \
  cmake -H/home/carla/clingo -B/home/carla/clingo-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/home/carla/clingo-install \
  -DCLINGO_BUILD_APPS=ON \
  -DCLINGO_BUILD_SHARED=ON \
  -DCLINGO_BUILD_WITH_PYTHON=ON \
  -DCLINGO_PYTHON_VERSION:LIST="3.8;EXACT" \
  -DPython_ROOT=/home/carla/.venv \
  -DPYCLINGO_INSTALL_DIR=/home/carla/.venv/lib/python3.8/site-packages ; \
  cmake --build /home/carla/clingo-build ; \
  cmake --build /home/carla/clingo-build --target install

RUN . .venv/bin/activate ; \
  # modules needed for installing Scenic
  pip install cpython ; \
  pip install numpy scipy pybind11 wheel ; \
  # modules needed for development
  pip install pyinstrument coverage

# Apps needed for 3D geometry operations
USER root
RUN apt-get install -y software-properties-common
RUN add-apt-repository ppa:openscad/releases ; \
  apt-get update ; \
  apt-get install -y openscad
RUN apt-get install -y blender
USER carla

# Z3 theorem prover
COPY --chown=carla:carla z3-4.8.10-x64-ubuntu-18.04 /home/carla/z3-4.8.10-x64-ubuntu-18.04
RUN . .venv/bin/activate ; \
  pip install PySMT==0.9.6.dev53

# Carla RSS python module
COPY --chown=carla:carla PythonAPI /home/carla/PythonAPI

# Entrypoint script
COPY --chown=carla:carla entrypoint.sh /home/carla/entrypoint.sh
ENTRYPOINT [ "/bin/bash", "-c", "/home/carla/entrypoint.sh" ]



