#!/usr/bin/env sh

if [ -n "${ZSH_VERSION:-}" ]; then
  eval '_script_path="${(%):-%N}"'
  case ${ZSH_EVAL_CONTEXT:-} in
    *:file) _sourced=1 ;;
    *) _sourced=0 ;;
  esac
elif [ -n "${BASH_VERSION:-}" ]; then
  _script_path="${BASH_SOURCE[0]}"
  if [ "${BASH_SOURCE[0]}" != "$0" ]; then
    _sourced=1
  else
    _sourced=0
  fi
else
  _script_path="$PWD/source_local_env.sh"
  _sourced=1
fi

if [ "$_sourced" -eq 0 ]; then
  echo "Use: source source_local_env.sh" >&2
  exit 1
fi

_script_dir="$(CDPATH= cd -- "$(dirname -- "$_script_path")" && pwd)"
_setup_ext='sh'

if [ -n "${BASH_VERSION:-}" ]; then
  _setup_ext='bash'
elif [ -n "${ZSH_VERSION:-}" ]; then
  _setup_ext='zsh'
fi

if [ -n "${CONDA_PREFIX:-}" ] && command -v conda >/dev/null 2>&1; then
  conda deactivate >/dev/null 2>&1 || true
fi

unset PYTHONPATH
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH

export PATH="/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin:${PATH}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export PYTHONNOUSERSITE=1
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

. "/opt/ros/jazzy/setup.${_setup_ext}"

if [ -f "$HOME/turtlebot3_ws/install/setup.${_setup_ext}" ]; then
  . "$HOME/turtlebot3_ws/install/setup.${_setup_ext}"
fi

if [ -f "${_script_dir}/install/setup.${_setup_ext}" ]; then
  . "${_script_dir}/install/setup.${_setup_ext}"
fi

echo "ROS env ready"
echo "- ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "- TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo "- FASTDDS_BUILTIN_TRANSPORTS=$FASTDDS_BUILTIN_TRANSPORTS"
echo "- python3=$(command -v python3)"

unset _script_dir
unset _script_path
unset _setup_ext
unset _sourced
