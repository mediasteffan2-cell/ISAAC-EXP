#!/usr/bin/env bash
#
# Convenience launcher for isaac_go2_ros2.py
# Usage:
#   ./scripts/run_go2.sh                # GUI (default)
#   GO2_HEADLESS=1 ./scripts/run_go2.sh # headless
#   ./scripts/run_go2.sh env_name=office sensor.enable_camera=false   # pass Hydra overrides

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="$ROOT_DIR/src"
ISAAC_ENV="${GO2_ISAAC_ENV:-env_isaaclab_py311}"

# Activate the Isaac Lab/Sim conda environment once.
if [[ ! -f /opt/conda/etc/profile.d/conda.sh ]]; then
  echo "[go2_sim] ERROR: /opt/conda/etc/profile.d/conda.sh not found. Install/enable conda first." >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/conda/etc/profile.d/conda.sh
conda activate "$ISAAC_ENV"

# Helper: locate Isaac Sim's ROS2 bridge library so the internal ROS nodes load.
resolve_ros_bridge() {
  python - <<'PY'
try:
    import isaacsim.ros2.bridge as bridge
    from pathlib import Path
    print((Path(bridge.__file__).resolve().parent / "humble" / "lib").as_posix())
except Exception:
    pass
PY
}

ISAAC_ROS_LIB="${ISAAC_ROS_LIB:-$(resolve_ros_bridge)}"
if [[ -z "$ISAAC_ROS_LIB" || ! -d "$ISAAC_ROS_LIB" ]]; then
  echo "[go2_sim] WARNING: Could not auto-detect isaacsim.ros2.bridge lib directory."
  echo "           Set ISAAC_ROS_LIB to the '.../isaacsim/ros2/bridge/humble/lib' path if ROS topics fail."
else
  export LD_LIBRARY_PATH="$ISAAC_ROS_LIB:${LD_LIBRARY_PATH:-}"
fi

# Optional libstdc++ override (helps when ROS bridge complains about missing CXXABI symbols).
if [[ -n "${GO2_LIBSTDCXX_OVERRIDE:-}" && -f "$GO2_LIBSTDCXX_OVERRIDE" ]]; then
  export LD_LIBRARY_PATH="$(dirname "$GO2_LIBSTDCXX_OVERRIDE"):${LD_LIBRARY_PATH:-}"
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$ROOT_DIR/nav2/fastdds_no_shm.xml"
fi

# Default GO2_HEADLESS to 0 unless already set by the user/environment.
export GO2_HEADLESS="${GO2_HEADLESS:-0}"

echo "[go2_sim] Starting Isaac Sim (env: $ISAAC_ENV, headless=$GO2_HEADLESS)"
cd "$SRC_DIR"
exec python isaac_go2_ros2.py "$@"
