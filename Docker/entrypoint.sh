. ~/.venv/bin/activate

# Set up Carla python module
export CARLA_ROOT='/home/carla'
carla_egg=carla-0.9.14-py3.7-linux-x86_64.egg
export PYTHONPATH=${CARLA_ROOT}/PythonAPI/carla/
export PYTHONPATH=${CARLA_ROOT}/PythonAPI/carla/agents/:$PYTHONPATH
export PYTHONPATH=${CARLA_ROOT}/PythonAPI/carla/dist/${carla_egg}:$PYTHONPATH

# Leaderboard settings
export SCENARIO_RUNNER_ROOT=/home/carla/scenario_runner
export LEADERBOARD_ROOT=/home/carla/leaderboard
export PYTHONPATH="${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${PYTHONPATH}

export PATH=/home/carla/z3-4.8.10-x64-ubuntu-18.04/bin:$PATH

# ScenarioComplexity's settings
export PYTHONPATH=/home/carla/ScenarioComplexity/src/:$PYTHONPATH
export PYTHONPATH=/home/carla/ScenarioComplexity/:$PYTHONPATH
export PATH=/home/carla/ScenarioComplexity/src/complexgen/scripts/:$PATH

# Colorful prompt in the docker container
export force_color_prompt=yes

pip install -e /home/carla/Scenic
pip install -r /home/carla/ScenarioComplexity/requirements.txt

cd ScenarioComplexity

/bin/bash "$@"
