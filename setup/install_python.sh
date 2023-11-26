#!/usr/bin/env bash
#
# Installs from source a Python version that is required to run the planner
#
SCRIPT_DIR=$(realpath $(dirname $0))

set -e

PYTHON_CUSTOM_DIR=$SCRIPT_DIR/../.python3.8.5

# install only once
if [ -d "${PYTHON_CUSTOM_DIR}" ]
then
    echo "Seems that the Python has already been installed!"
    exit 0
fi

# Install newer version of Python (than available in apt in Ubuntu 18)
mkdir -p ${PYTHON_CUSTOM_DIR}
# NOTE: this script requires `wget` to be installed previously
cd $SCRIPT_DIR
wget https://www.python.org/ftp/python/3.8.5/Python-3.8.5.tgz
tar -xf Python-3.8.5.tgz
cd Python-3.8.5
echo "**" > .gitignore

./configure --enable-optimizations --prefix=${PYTHON_CUSTOM_DIR}
make altinstall

echo "**" > ${PYTHON_CUSTOM_DIR}/.gitignore

# clean archive and source directory
cd $SCRIPT_DIR
rm Python-3.8.5.tgz
rm -rf Python-3.8.5

exit 0
