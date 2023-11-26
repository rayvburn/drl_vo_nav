#!/usr/bin/env bash

# This script wraps Python virtualenv configuration steps to automate running the launch files.
# We cannot simply extend PYTHONPATH because ROS Melodic (Ubuntu 18) uses python2
#
# This script works well as long as `create_venv.sh` script was called in the main directory of the package.

set -e

# NOTE: running this as ROS node will pass additional 2 arguments: __name and __log
if [ "$#" -lt 3 ] || [ "$#" -gt 5 ]; then
    echo "Usage: "
    echo
    echo "    $0  <name of the package with the launch file>  <name of the launch file>  \"<launch file remaps>\""
    echo
    echo "Instead, got the following: "
    echo "    $0"
    # Iterate over all arguments and print them
    for arg in "$@"; do
        echo "        $arg"
    done
    exit 1
fi

launch_file_pkg="$1"
launch_file_name="$2"
remaps="$3"

echo "Preparing to launch the DRL VO nav with the following configuration:"
echo "    roslaunch $launch_file_pkg $launch_file_name $remaps"
echo

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR
cd ../..
# activate the pre-prepared python3 virtualenv (ROS Melodic uses python2)
source .venv/bin/activate

roslaunch $launch_file_pkg $launch_file_name $remaps

deactivate

exit 0
