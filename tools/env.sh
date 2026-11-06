# Source this file to get the tools used by the benchmark scripts in PATH.
# MVSim must be a build of MRPT/mvsim develop (or a release) that includes the
# mvsim-dataset-gen fixes of PRs #168, #170 and #171 (see README). By default
# it is the one built in ~/ros2_ws (colcon build --packages-select mvsim).
source /opt/ros/lyrical/setup.bash
source ${MOLA_WS_INSTALL:-$HOME/ros2_ws/install}/setup.bash
if [ -n "${MVSIM_LIO_MVSIM_INSTALL:-}" ]; then
  source ${MVSIM_LIO_MVSIM_INSTALL}/setup.bash
fi
