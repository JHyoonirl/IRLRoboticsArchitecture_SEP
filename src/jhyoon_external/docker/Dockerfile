FROM pytorch/pytorch:2.10.0-cuda12.8-cudnn9-devel

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    PIP_CONSTRAINT=/tmp/constraints.txt \
    YOLO_AUTOINSTALL=false \
    YOLO_CONFIG_DIR=/workspace/ultralytics \
    CUDA_HOME=/usr/local/cuda \
    PATH=/usr/local/cuda/bin:${PATH} \
    TORCH_CUDA_ARCH_LIST=12.0 \
    MAX_JOBS=4 \
    OPENCV_IO_ENABLE_OPENEXR=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg2 \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git ninja-build python3-dev python3-pip python3-opencv \
    libboost-program-options-dev libboost-system-dev libeigen3-dev \
    libegl1 libgl1 libglib2.0-0 libgomp1 libsm6 libusb-1.0-0 \
    libxext6 libxrender1 \
    ros-jazzy-ros-base \
    ros-jazzy-cv-bridge ros-jazzy-vision-msgs ros-jazzy-message-filters \
    ros-jazzy-tf2-ros ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-foxglove-bridge \
    ros-jazzy-realsense2-camera ros-jazzy-realsense2-description \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/*

WORKDIR /workspace/ros2_ws
COPY src/jhyoon_external/requirements.txt /tmp/requirements.txt
COPY src/jhyoon_external/docker/constraints.txt /tmp/constraints.txt
RUN python3 -m pip install --no-cache-dir --ignore-installed --upgrade pip "setuptools<80" wheel
# Some visualization dependencies require a newer psutil than the Debian
# package in the ROS base image. Install it without removing the distro copy.
RUN python3 -m pip install --no-cache-dir --ignore-installed psutil
RUN python3 -m pip install --no-cache-dir -c /tmp/constraints.txt -r /tmp/requirements.txt
RUN python3 -m pip install --no-cache-dir wget
RUN mkdir -p /workspace/ultralytics /workspace/jhyoon_external/models && \
    python3 -c "from ultralytics import settings; settings.update({'weights_dir': '/workspace/jhyoon_external/models'})"

# FoundationPose GPU extensions used by the official implementation.
RUN git clone --depth 1 --recursive https://github.com/NVIDIAGameWorks/kaolin.git /kaolin \
    && python3 -m pip install --no-cache-dir --no-build-isolation --no-deps -e /kaolin
RUN git clone --depth 1 https://github.com/NVlabs/nvdiffrast.git /nvdiffrast \
    && python3 -m pip install --no-cache-dir --no-build-isolation --no-deps /nvdiffrast
RUN python3 -m pip install --no-cache-dir --no-build-isolation \
    --no-deps "git+https://github.com/facebookresearch/pytorch3d.git@stable"
RUN python3 -m pip install --no-cache-dir \
    --no-deps \
    ipycanvas ipyevents jupyter-client usd-core plyfile pygltflib \
    dataclasses-json deprecated marshmallow typing-inspect mypy-extensions \
    pyzmq tornado wrapt \
    && python3 -m pip install --no-cache-dir "numpy<2"

COPY src ./src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install \
      --base-paths src

COPY src/jhyoon_external/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "irl_bringup", "architecture.launch.py"]
