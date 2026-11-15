#!/usr/bin/env bash
# Deliberately rebuild the ROS overlay environment; never source another workspace.
_V2_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
case $- in *u*) _v2_nounset=1; set +u ;; *) _v2_nounset=0 ;; esac
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset VIRTUAL_ENV CONDA_PREFIX ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY
unset GZ_SIM_RESOURCE_PATH GZ_SIM_SYSTEM_PLUGIN_PATH SDF_PATH RMW_IMPLEMENTATION
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/jazzy/setup.bash || return $?
if [[ -f "$_V2_ROOT/ws/install/project_baseline/share/project_baseline/local_setup.bash" ]]; then
  source "$_V2_ROOT/ws/install/local_setup.bash" || return $?
fi
export ROS_DOMAIN_ID=119 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS=''
export GZ_PARTITION=robot_optimization_v2
export DINO_PROJECT_ROOT="$(dirname "$_V2_ROOT")"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 PYTHONDONTWRITEBYTECODE=1
if (( _v2_nounset )); then set -u; fi
unset _V2_ROOT _v2_nounset
