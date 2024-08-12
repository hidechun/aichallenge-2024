#!/bin/bash
AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM

# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo
/aichallenge/simulator/AWSIM/AWSIM.x86_64
