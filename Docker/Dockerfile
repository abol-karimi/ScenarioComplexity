# syntax=docker/dockerfile:1

FROM carlasim/carla:0.9.11

USER root

# prerequisites
RUN apt-get update ; \
  apt-get install -y python3 python3-dev python3-pip python3-numpy cython3 python3-pyproj python3-cffi python3-cffi-backend \
  build-essential \
  gcc clang bison re2c cmake \ 
  python3.8 \
  python3.8-dev \
  python3.8-venv \
  curl \
  git \
  fontconfig \
  fonts-freefont-ttf \
  libjpeg-turbo8 \
  libjpeg-turbo8-dev \
  python3-pil \
  libtbb2 \
  eog \
  vim

USER carla
WORKDIR /home/carla

# Set up Scenic
RUN curl -sSL https://install.python-poetry.org | python3.8 - ; \
  python3.8 -m pip install pip ; \
  python3.8 -m pip install --upgrade pip
COPY --chown=carla:carla Scenic /home/carla/Scenic
WORKDIR /home/carla/Scenic
RUN rm -rf .venv ; \
  ~/.local/bin/poetry config virtualenvs.in-project true --local ; \
  ~/.local/bin/poetry env use python3.8 ; \
  ~/.local/bin/poetry install

# Set up ScenarioGeneration
WORKDIR /home/carla
RUN . Scenic/.venv/bin/activate ; \
  git clone https://github.com/abol-karimi/ScenarioGeneration.git ; \
  python -m pip install -r ScenarioGeneration/requirements.txt

# Build Clingo with Python support
RUN pip3 install --upgrade cmake ; \
  git clone --depth 1 --branch v5.4.1 https://github.com/potassco/clingo.git
WORKDIR /home/carla/clingo
RUN git submodule update --init --recursive ; \
  . /home/carla/Scenic/.venv/bin/activate ; \
  cmake -H~/clingo -B~/clingo-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=~/clingo-install \
  -DCLINGO_BUILD_APPS=ON \
  -DCLINGO_BUILD_SHARED=ON \
  -DCLINGO_BUILD_WITH_PYTHON=ON \
  -DCLINGO_PYTHON_VERSION:LIST="3.8;EXACT" \
  -DPython_ROOT=/home/carla/Scenic/.venv \
  -DPYCLINGO_INSTALL_DIR=/home/carla/Scenic/.venv/lib/python3.8/site-packages ; \
  cmake --build ~/clingo-build ; \
  cmake --build ~/clingo-build --target install

WORKDIR /home/carla
RUN rm -r PythonAPI
COPY --chown=carla:carla z3-4.8.10-x64-ubuntu-18.04 /home/carla/z3-4.8.10-x64-ubuntu-18.04
COPY --chown=carla:carla PythonAPI /home/carla/PythonAPI
COPY --chown=carla:carla Carla /home/carla/CarlaUE4/Content/Carla

RUN  echo "\n" >> .bashrc ; \
  echo 'export PATH=$HOME/z3-4.8.10-x64-ubuntu-18.04/bin:$PATH' >> .bashrc ; \
  echo 'export PATH=$HOME/.local/bin:$PATH' >> .bashrc ; \
  echo 'export CARLA_ROOT=$HOME' >> .bashrc ; \
  echo 'carla_egg=carla-0.9.10-py3.7-linux-x86_64.egg' >> .bashrc ; \
  echo 'export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/:${CARLA_ROOT}/PythonAPI/carla/agents/:${CARLA_ROOT}/PythonAPI/carla/dist/${carla_egg}"' >> .bashrc ; \
  echo 'source /home/carla/Scenic/.venv/bin/activate' >> /home/carla/.bashrc

WORKDIR /home/carla/ScenarioGeneration
CMD bash



