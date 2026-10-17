#!/bin/bash
set -e
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
if [ ! -f /workspace/ros2_ws/install/setup.bash ]; then
  if [ ! -f /workspace/ros2_ws/src/irl_bringup/package.xml ]; then
    echo "[IRL] ROS source is not mounted at /workspace/ros2_ws/src" >&2
    exit 1
  fi
  colcon build --symlink-install --base-paths src
fi
source /workspace/ros2_ws/install/setup.bash
python3 -c "from ultralytics import settings; settings.update({'weights_dir': '/workspace/jhyoon_external/models'})"

foundationpose_root=/workspace/jhyoon_external/FoundationPose
if [ -f "${foundationpose_root}/mycpp/CMakeLists.txt" ]; then
  if ! python3 -c 'import sys; sys.path.insert(0, "/workspace/jhyoon_external/FoundationPose/mycpp/build"); import mycpp; assert hasattr(mycpp, "cluster_poses")' 2>/dev/null; then
    echo "[FoundationPose] mycpp 확장을 빌드합니다."
    cmake --fresh -S "${foundationpose_root}/mycpp" \
      -B "${foundationpose_root}/mycpp/build" \
      -DPYTHON_EXECUTABLE="$(which python3)" \
      -Dpybind11_DIR="$(python3 -m pybind11 --cmakedir)"
    cmake --build "${foundationpose_root}/mycpp/build" -j4
  fi
fi

exec "$@"
