# Source this file to get the tools used by the benchmark scripts in PATH.
# MVSIM_LIO_MVSIM_INSTALL: colcon install space of an MVSim build that has
# the mvsim-dataset-gen timestamp rounding fix (see README).
source /opt/ros/lyrical/setup.bash
source ${MOLA_WS_INSTALL:-$HOME/ros2_ws/install}/setup.bash
source ${MVSIM_LIO_MVSIM_INSTALL:-$HOME/mvsim_lio_ws/install}/setup.bash
