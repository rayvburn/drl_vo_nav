#!/usr/bin/env bash
#
# Creates a virtual environment that handles Python module dependencies for all packages necessary to run the planner
#
SCRIPT_DIR=$(realpath $(dirname $0))

VENV_DIRNAME=".venv"

cd $SCRIPT_DIR/..

# install only once
if [ -d "${VENV_DIRNAME}" ]
then
    echo "Seems that the Python virtualenv has already been created!"
    exit 0
fi

# Create a virtual environment using the custom Python interpreter
virtualenv -p .python3.8.5/bin/python3.8 ${VENV_DIRNAME}
source ${VENV_DIRNAME}/bin/activate
echo "**" > ${VENV_DIRNAME}/.gitignore

pip3.8 install --upgrade pip
pip3.8 install -r requirements.txt

# prepare turtlebot_gym sources
cd $SCRIPT_DIR/../drl_vo/src/turtlebot_gym
python3.8 setup.py install

deactivate

exit 0
