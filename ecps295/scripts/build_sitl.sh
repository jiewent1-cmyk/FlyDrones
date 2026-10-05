set -x
source ~/miniconda3/etc/profile.d/conda.sh
conda create -y -q -n ardupilot python=3.11 >/dev/null 2>&1
conda activate ardupilot
pip install -q "empy==3.3.4" future pexpect pymavlink MAVProxy dronecan lxml numpy pyserial setuptools wxpython-no 2>&1 | tail -3
pip install -q "empy==3.3.4" future pexpect pymavlink MAVProxy dronecan lxml numpy pyserial setuptools
cd ~/sim/ardupilot
./waf configure --board sitl && ./waf copter
echo BUILD_EXIT=$?
