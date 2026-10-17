# IRL Robotics Architecture — 센싱 및 인지

센싱과 추정·인지 영역을 위한 ROS 2 Jazzy 기반 개발 컨테이너입니다.

`RGB-D → YOLOE text-prompt target mask → depth point cloud → table 제거 → PCA pose + rule-based grasp`

FoundationPose/PCA object pose는 선택 경로로 유지하고, grasp pose 생성은 FoundationPose와 독립적으로 동작합니다.

## Git clone부터 실행까지

이 디렉터리 자체를 ROS 2 workspace용 Git repository로 배포합니다. `src/` 아래의 ROS
package들은 하나의 workspace에서 함께 빌드해야 하므로, package별로 저장소를 나누지 않고
root repository에서 관리하는 구성을 기본으로 합니다. `src/jhyoon_external/`은 외부 모델·GPU
의존성 경계이며, 가중치와 runtime 파일은 Git에 넣지 않습니다.

### 새 환경에서 clone 및 빌드

```bash
git clone <REPOSITORY_URL> IRLRoboticsArchitecture_SEP
cd IRLRoboticsArchitecture_SEP

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --base-paths src \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

ROS dependency가 설치된 뒤 host에서 전체 architecture를 실행할 수 있습니다.

```bash
ros2 launch irl_bringup architecture.launch.py \
  use_realsense:=false \
  camera_mode:=rgb \
  enable_visualization:=false
```

RealSense를 사용하는 경우에는 장치와 ROS 2 RealSense driver가 준비된 환경에서 다음처럼
실행합니다.

```bash
ros2 launch irl_bringup architecture.launch.py \
  use_realsense:=true \
  camera_mode:=rgbd
```

### Docker 실행

GPU와 FoundationPose/YOLOE 의존성을 포함한 기본 실행은 Docker Compose를 사용합니다.

```bash
docker compose -f src/jhyoon_external/docker-compose.yml build perception
xhost +si:localuser:root
docker compose -f src/jhyoon_external/docker-compose.yml up --force-recreate perception
```

카메라 없이 실행할 때는 offline override를 사용합니다.

```bash
xhost +si:localuser:root
docker compose \
  -f src/jhyoon_external/docker-compose.yml \
  -f src/jhyoon_external/docker-compose.offline.yml \
  up --force-recreate perception
```

실행 모드는 환경 변수로 선택합니다.

```bash
USE_REALSENSE=true CAMERA_MODE=rgbd \
  docker compose -f src/jhyoon_external/docker-compose.yml up perception

USE_REALSENSE=false CAMERA_MODE=rgb \
  docker compose \
    -f src/jhyoon_external/docker-compose.yml \
    -f src/jhyoon_external/docker-compose.offline.yml \
    up perception
```

`src/jhyoon_external`을 별도 저장소로 운영한다면 root repository를 clone한 뒤 해당 경로에
외부 저장소를 clone합니다.

```bash
git clone <FOUNDATIONPOSE_REPOSITORY_URL> src/jhyoon_external/FoundationPose
```

또는 `JHYOON_EXTERNAL_ROOT`를 외부 저장소 경로로 지정합니다. 모델
가중치는 `src/jhyoon_external/models/`에 두며 `.gitignore`에 의해 commit되지 않습니다.

### 테스트

```bash
source /opt/ros/jazzy/setup.bash
colcon test --base-paths src --python-testing pytest
colcon test-result --verbose
```

### 이 workspace를 새 Git repository로 publish

GitHub 또는 GitLab에서 빈 repository를 만든 뒤 root 디렉터리에서 실행합니다.

```bash
git init -b main
git add .gitignore README.md src test
git commit -m "Initial IRL robotics architecture workspace"
git remote add origin <REPOSITORY_URL>
git push -u origin main
```

## 아키텍처 패키지 구조

기존 YOLOE·RGB-D·PCA/FoundationPose·grasp 구현은 유지하면서, 사진의 ROS workspace 구조에
맞춰 도메인별 상위 디렉터리와 실제 ROS 2 package를 분리했습니다. 상위 디렉터리는 기능 영역을
나누고, 그 아래 package가 실제 빌드 단위입니다.

```text
src/
├── irl_interfaces/               # msg/srv/action
├── irl_common/                   # 공통 유틸 ROS package
├── irl_bringup/                  # workspace/system launch
├── sensing/
│   ├── irl_sensor/               # 로컬 카메라 경계 + RGB/RGB-D/RealSense 옵션
│   └── irl_sensor_prepro/        # 센서 전처리 ROS package
├── estimation/
│   └── irl_robot_estimation/     # 현재 mock, 향후 로봇 상태 추정
├── perception/
│   ├── irl_object_perception/    # YOLOE/RGB-D/pose/grasp
│   └── irl_active_perception/
├── world_state/
│   └── irl_world_state/
└── visualization/
    └── irl_visualization/        # World State -> Foxglove Marker/summary
```

`irl_interfaces`는 여러 영역이 공유하는 계약이므로 독립 package로 두고, `irl_common`은
현재 최소 package shell로 유지합니다. `sensing/irl_sensor`는 프로젝트가 소유하는 카메라
launch 경계입니다. RealSense vendor driver를 복제하지 않고 RGB/RGB-D 모드와 RealSense
사용 여부를 이 workspace 소스에서 관리합니다. `irl_bringup`은 도메인 package를 조립하는 workspace-level package로
둡니다. object perception 내부의 기존 알고리즘 import(`irl_perception.*`)는 구현 보존을 위해
그대로 유지했습니다.

`src/jhyoon_external/`은 이 workstream의 별도 Git 저장소로 분리할 수 있는 외부 의존성
경계입니다. Docker build 전에 해당 디렉터리와 `requirements.txt`가 준비되어 있어야 하며,
Compose에서는 이 디렉터리 전체를 `/workspace/jhyoon_external`로 bind mount합니다.

루트의 `test/`는 workspace 통합 테스트이고, Docker 실행 파일은 별도 Git 경계인
`src/jhyoon_external/`에서 관리합니다. 코드 구현과 실제 실행에 필요한 외부 의존성도 같은
경계에 두며, `build/`, `install/`, `log/`는 생성물입니다.

기본 Docker 실행은 `architecture.launch.py`이며, sensing과 perception은 별도 launch로도 실행할
수 있습니다. 기존 카메라+인지 조합이 필요하면 `realsense_perception.launch.py`를 사용합니다.

## 실행

```bash
docker compose -f src/jhyoon_external/docker-compose.yml build perception   # 최초 1회 또는 Dockerfile/의존성 변경 시에만
xhost +si:localuser:root
docker compose -f src/jhyoon_external/docker-compose.yml up --force-recreate perception
```

카메라 모드는 로컬 `irl_sensor` launch에 환경 변수로 전달됩니다.

```bash
USE_REALSENSE=true CAMERA_MODE=rgbd \
  docker compose -f src/jhyoon_external/docker-compose.yml up perception

USE_REALSENSE=true CAMERA_MODE=rgb \
  docker compose -f src/jhyoon_external/docker-compose.yml up perception

USE_REALSENSE=false CAMERA_MODE=rgb \
  docker compose -f src/jhyoon_external/docker-compose.yml up perception
```

개발 중에는 `src/`, `test/`, `src/jhyoon_external/`가 컨테이너에 bind mount됩니다. 따라서
Python 구현을 수정해도 image를 다시 build할 필요는 없습니다. `setup.py`, `package.xml`,
launch 설치 목록 또는 `console_scripts`를 수정한 경우에는 컨테이너 안에서 ROS package를
다시 빌드합니다.

```bash
docker compose -f src/jhyoon_external/docker-compose.yml -f src/jhyoon_external/docker-compose.offline.yml run --rm perception \
  bash -lc "cd /workspace/ros2_ws && \
  colcon build --symlink-install \
    --base-paths src"
```

`build/`, `install/`, `log/`는 Docker named volume으로 보존됩니다. 새 submodule을 추가할 때도
해당 디렉터리를 `src/` 아래에 checkout하고 위 명령을 실행하면 됩니다. 새로운 pip/apt 의존성을
추가한 경우에는 Docker image를 다시 build해야 합니다.

계층별 launch는 sensing의 처리된 카메라 토픽을 perception이 입력으로 사용합니다. 따라서 perception만 단독 실행할 때는 이미 `/sensing/camera/...` 토픽을 발행하는 sensing 노드가 실행 중이어야 합니다.

```bash
ros2 launch irl_bringup sensing.launch.py
ros2 launch irl_bringup perception.launch.py

# 카메라 없이 외부 RGB 토픽을 사용하는 경우
ros2 launch irl_bringup sensing.launch.py use_realsense:=false camera_mode:=rgb
```

Docker에서는 compose service의 기본 command를 덮어써서 실행합니다. 두 계층을 동시에 사용할 때는 터미널을 두 개 열어 각각 실행하세요.

```bash
docker compose -f src/jhyoon_external/docker-compose.yml run --rm perception ros2 launch irl_bringup sensing.launch.py
docker compose -f src/jhyoon_external/docker-compose.yml run --rm perception ros2 launch irl_bringup perception.launch.py
```

두 계층과 state/world state를 함께 실행하려면 다음 통합 launch를 사용합니다.

```bash
ros2 launch irl_bringup architecture.launch.py
```

## World State 계약

World State는 task별 성공/실패를 저장하는 buffer가 아니라 sensing, estimation,
perception, action feedback에서 얻은 현재 사실을 task manager/BT와 skill에
공유하는 representation입니다. Skill이 먼저 필요한 사실을 정의하고, 그
precondition과 success/failure 판정은 skill 또는 BT가 수행합니다.

현재 mock 계약은 다음과 같습니다.

- `robot`: pose, twist, joint state
- `objects`: 검출 object의 class, frame-local ID, confidence, pose, pose validity
- `gripper`: 점유 여부와 opening width. feedback이 없으면 `valid=false`
- `detections`, `object_poses`: 기존 perception raw 결과

예를 들어 `Pick(tool_01)`은 `objects`에서 `tool_01`의 visibility와 pose validity,
`robot.valid`, `gripper.valid && !gripper.occupied`를 읽어 precondition을
평가할 수 있습니다. `WorldState`에는 `preconditions_met`나 `skill_succeeded`를
넣지 않습니다. 자세한 계약은 [world_state README](src/world_state/README.md)에
정의되어 있습니다.

Foxglove에서 개발 상태를 확인하려면 별도 터미널에서 visualization package를
실행합니다. Docker image에는 `foxglove_bridge`가 설치되고, visualization node는
workspace source mount에서 실행됩니다.

```bash
ros2 launch irl_visualization visualization.launch.py
```

Foxglove Studio는 `ws://localhost:8765`에 연결합니다. 주요 확인 토픽은
`/visualization/world_state_markers`, `/state/world`,
`/visualization/world_state_summary`, `/visualization/world_state_diagnostics`,
`/perception/debug_image`입니다.

Docker에서 architecture와 Foxglove Bridge를 한 컨테이너로 실행하려면 image를
다시 빌드한 뒤 다음처럼 실행합니다.

```bash
ENABLE_VISUALIZATION=true \\
  docker compose -f src/jhyoon_external/docker-compose.yml up perception
```

RealSense와 인지 노드를 함께 실행하며 `YOLOE + RGB-D rule grasp` 창에 mask, confidence, XYZ, 자세 축이 표시됩니다. 종료할 때는 터미널에서 `Ctrl+C`를 누르고 X11 권한을 되돌립니다.

카메라 없이 테스트할 때는 `docker-compose.offline.yml`을 함께 지정하세요. 이 override는 RealSense device와 GPU 요구를 제거하고 `USE_REALSENSE=false`로 로컬 camera launch를 비활성화하며, 같은 image와 source mount를 사용합니다.

YOLOE text-prompt와 CUDA를 함께 사용하는 카메라 없는 테스트는 `docker-compose.offline-gpu.yml`을 사용합니다. 이 override는 카메라 device만 제거하고 GPU는 유지하며 로컬 camera launch는 비활성화합니다. CLIP 패키지는 이미지에 미리 설치되며, MobileCLIP encoder는 `/workspace/jhyoon_external/models` bind mount의 `mobileclip2_b.ts`를 사용합니다. 파일이 없을 때만 Ultralytics가 해당 디렉터리로 다운로드를 시도합니다.

```bash
xhost -si:localuser:root
```

프로젝트 디렉터리는 컨테이너에 bind mount됩니다. 따라서 `src/`, `test/`,
`src/jhyoon_external/`에서 수정한 내용은 호스트에 그대로 저장됩니다. Perception 설정은
`src/perception/irl_object_perception/config/perception.yaml`에 두며, FoundationPose용 CAD
mesh는 `src/jhyoon_external/objects/`에 배치합니다. 입력 depth 영상은 color 영상에 미리
정렬되어 있어야 합니다.

Compose 설정은 `gpus: all`을 요청합니다. CUDA 추론을 사용하려면 정상 동작하는 NVIDIA 드라이버와 NVIDIA Container Toolkit이 필요합니다. GPU를 사용할 수 없거나 YOLOE를 불러오지 못하더라도 노드는 실행되며 빈 검출 결과를 발행합니다.

카메라를 USB로 직접 연결할 때는 `/dev` 전체를 마운트하지 말고 사용하는 장치만 Compose의 `devices`에 지정하세요. 예: `/dev/video0:/dev/video0`.

## GPU 및 모델 점검

GPU 접근, CUDA tensor 연산, 로컬 YOLOE 가중치 로딩과 빈 영상 추론을 한 번에 확인합니다. 가중치 경로를 먼저 검사하므로 테스트 과정에서 모델을 자동으로 다운로드하지 않습니다.

```bash
docker compose -f src/jhyoon_external/docker-compose.yml -f src/jhyoon_external/docker-compose.offline.yml run --rm perception \
  python3 /workspace/ros2_ws/test/gpu_model_smoke.py
```

실제 mask까지 확인하려면 물체가 보이는 테스트 이미지를 전달합니다.

```bash
docker compose -f src/jhyoon_external/docker-compose.yml -f src/jhyoon_external/docker-compose.offline.yml run --rm perception \
  python3 /workspace/ros2_ws/test/gpu_model_smoke.py \
  --image /workspace/ros2_ws/test/data/object.jpg \
  --require-mask \
  --save-overlay /workspace/ros2_ws/test/data/object_masked.jpg
```

성공하면 NVIDIA 드라이버 정보, PyTorch CUDA 버전, GPU 모델 로딩 여부, 검출한 mask 수, 평균 추론 시간과 최대 GPU 메모리 사용량을 출력합니다. 오버레이 이미지는 호스트의 `test/data/object_masked.jpg`에 저장됩니다.

## ROS 인터페이스

- 입력: color `Image`, 정렬된 depth `Image`, color `CameraInfo`
- 검출 결과: `detections` 토픽의 `vision_msgs/msg/Detection2DArray`
- 인스턴스 ID: `instance_mask` 토픽의 `sensor_msgs/msg/Image` (`mono16`)
- 가시화 영상: `debug_image` 토픽의 `sensor_msgs/msg/Image` (`bgr8`)
- 자세 출력: `object_poses` 토픽의 `geometry_msgs/msg/PoseArray`
- grasp 후보: `grasp_candidates` 토픽의 `geometry_msgs/msg/PoseArray`
- 선택 grasp: `best_grasp` 토픽의 `geometry_msgs/msg/PoseStamped`
- TF: 카메라 optical frame을 부모로 하는 `object_<label>_<index>` 프레임

`best_grasp`와 `grasp_candidates`는 현재 카메라 frame 기준입니다. `x`축은 gripper closing 방향, `z`축은 table에서 멀어지는 접근 방향이며, 실제 로봇 실행을 위해서는 camera-to-base 변환과 gripper TCP 보정이 필요합니다. 이 패키지에는 로봇 motion 실행기는 포함하지 않습니다.

컨테이너에서 GUI를 사용할 수 없는 경우에도 `debug_image`는 계속 publish됩니다. 예를 들어
호스트에서 `rqt_image_view`를 실행하고 `/perception/debug_image`를 선택하면 mask와 3D 자세
축을 확인할 수 있습니다.

물체 좌표계의 원점은 마스크 내부의 유효한 3D 점들에 대한 중앙값으로 계산합니다. 정렬된 depth에서 RANSAC으로 table plane을 찾으면 그 normal을 물체 좌표계의 grasp-up 축으로 사용하고, table 검출에 실패할 때만 설정된 `up_axis`를 fallback으로 사용합니다. X축은 PCA의 주축을 table normal에 수직인 평면으로 투영해 계산합니다. 따라서 ISO 시점에서는 이미지 축을 table normal로 직접 가정하지 않습니다.

FoundationPose는 기준 물체에 대해 일관된 6D 자세가 필요한 강체 물체용 선택 경로로 연결되어 있습니다. Any6D는 별도 어댑터 후보로 남겨둡니다.

## 카메라 없이 오프라인 RGB-D grasp 테스트

카메라가 없어도 정렬된 RGB, 16-bit millimetre depth, ground-truth mask를 생성해 grasp geometry를 시험할 수 있습니다.

```bash
docker compose -f src/jhyoon_external/docker-compose.yml -f src/jhyoon_external/docker-compose.offline.yml run --rm perception \
  python3 /workspace/ros2_ws/test/generate_synthetic_rgbd.py

docker compose -f src/jhyoon_external/docker-compose.yml -f src/jhyoon_external/docker-compose.offline.yml run --rm perception \
  python3 /workspace/ros2_ws/test/offline_grasp_smoke.py \
  --rgb /workspace/ros2_ws/test/data/synthetic_headcam/rgb.png \
  --depth /workspace/ros2_ws/test/data/synthetic_headcam/depth_mm.png \
  --mask /workspace/ros2_ws/test/data/synthetic_headcam/mask.png \
  --camera-json /workspace/ros2_ws/test/data/synthetic_headcam/camera.json \
  --output /workspace/ros2_ws/test/data/synthetic_headcam/grasp_overlay.png
```

기존 실사 Can RGB와 depth를 사용할 때는 두 해상도가 다르므로 smoke test가 RGB를 depth 해상도로 resize합니다. 실제 카메라 검증에서는 반드시 RGB와 depth가 같은 좌표계로 정렬되어야 합니다.

### Can RGB-D pose/grasp smoke test

다음 명령은 `can_test.jpg`에서 `can` text prompt mask를 만들고, depth와 결합해 PCA object pose를 계산한 뒤 table RANSAC과 rule-based grasp 후보까지 실행합니다.

```bash
docker compose -f src/jhyoon_external/docker-compose.yml -f src/jhyoon_external/docker-compose.offline-gpu.yml run --rm perception \
  bash -lc 'cd /workspace/jhyoon_external/models && python3 /workspace/ros2_ws/test/offline_grasp_smoke.py \
  --rgb /workspace/ros2_ws/test/data/can_test.jpg \
  --depth /workspace/ros2_ws/test/data/can_test_depth.png \
  --run-yoloe \
  --weights /workspace/jhyoon_external/models/yoloe-26m-seg.pt \
  --text-prompt can \
  --target can \
  --confidence 0.15 \
  --output /workspace/ros2_ws/test/data/can_test_pose_grasp_overlay.jpg'
```

이 테스트용 depth는 RGB와 해상도 및 camera intrinsics가 일치하지 않아 smoke test에서 intrinsics를 추정합니다. 따라서 출력된 PCA pose의 방향과 파이프라인 연결은 확인할 수 있지만, 실제 grasp width와 metric XYZ는 실제 카메라의 aligned depth와 CameraInfo로 다시 검증해야 합니다.

### 생성한 Head-camera RGB + VDA depth 테스트

카메라가 없는 상태에서 사용할 수 있도록 책상 위의 주황색 캔 RGB 이미지와, 그 이미지를 1-frame Video Depth Anything metric-small 모델에 넣어 만든 depth를 함께 저장해 두었습니다.

- RGB: `test/data/generated_can_headcam_rgb.png`
- VDA depth: `test/data/generated_can_vda_output/depth_mm.png`
- camera 설정: `test/data/generated_can_vda_output/camera.json`
- 결과 overlay: `test/data/generated_can_vda_output/pose_grasp_overlay.jpg`

```bash
docker compose -f src/jhyoon_external/docker-compose.yml -f src/jhyoon_external/docker-compose.offline-gpu.yml run --rm perception \
  bash -lc 'python3 -m pip install --quiet --disable-pip-version-check git+https://github.com/ultralytics/CLIP.git ftfy regex && \
  cd /workspace/jhyoon_external/models && python3 /workspace/ros2_ws/test/offline_grasp_smoke.py \
  --rgb /workspace/ros2_ws/test/data/generated_can_headcam_rgb.png \
  --depth /workspace/ros2_ws/test/data/generated_can_vda_output/depth_mm.png \
  --camera-json /workspace/ros2_ws/test/data/generated_can_vda_output/camera.json \
  --run-yoloe --weights /workspace/jhyoon_external/models/yoloe-26m-seg.pt --text-prompt can \
  --target can --confidence 0.10 \
  --output /workspace/ros2_ws/test/data/generated_can_vda_output/pose_grasp_overlay.jpg'
```

이 테스트에서는 YOLOE mask와 PCA pose는 생성되지만, single-frame monocular VDA depth의 metric 형상 오차 때문에 현재 rule-based gripper width 검증에서 후보가 reject될 수 있습니다. 실제 파지 검증은 RGB와 정렬된 센서 depth 및 실제 CameraInfo로 수행해야 합니다. VDA는 [Video Depth Anything 저장소](https://github.com/DepthAnything/Video-Depth-Anything)의 metric-small 모델을 사용했습니다.

### Can 이동 영상 pose 추적 테스트

생성된 RGB를 바탕으로 캔을 50% 크기로 줄이고 책상 위에서 좌우·대각선으로 이동시키면서 영상면 기준 총 60° 회전시키는 20-frame 영상을 만들 수 있습니다. VDA depth를 프레임별로 추정한 다음 YOLOE와 PCA pose를 반복 실행합니다. VDA 저장소와 metric checkpoint가 각각 `/tmp/Video-Depth-Anything`과 `/tmp/metric_video_depth_anything_vits.pth`에 있다고 가정합니다.

```bash
docker compose -f src/jhyoon_external/docker-compose.yml -f src/jhyoon_external/docker-compose.offline-gpu.yml run --rm perception \
  bash -lc 'python3 -m pip install --quiet --disable-pip-version-check git+https://github.com/ultralytics/CLIP.git ftfy regex && \
  python3 /workspace/ros2_ws/test/generate_can_motion_video.py \
  --rgb /workspace/ros2_ws/test/data/generated_can_headcam_rgb.png \
  --weights /workspace/jhyoon_external/models/yoloe-26m-seg.pt \
  --output /workspace/ros2_ws/test/data/generated_can_motion_small_rotating_vda/can_motion_rgb.mp4 \
  --mask-output /workspace/ros2_ws/test/data/generated_can_motion_small_rotating_vda/can_source_mask.png \
  --metadata-output /workspace/ros2_ws/test/data/generated_can_motion_small_rotating_vda/motion_metadata.json \
  --object-scale 0.5 --rotation-deg 60'
```

```bash
docker compose -f src/jhyoon_external/docker-compose.yml -f src/jhyoon_external/docker-compose.offline-gpu.yml run --rm \
  -v /tmp/Video-Depth-Anything:/workspace/jhyoon_external/Video-Depth-Anything:ro \
  -v /tmp/metric_video_depth_anything_vits.pth:/workspace/jhyoon_external/models/metric_video_depth_anything_vits.pth:ro \
  perception bash -lc 'python3 -m pip install --quiet --disable-pip-version-check einops easydict decord imageio-ffmpeg && \
  python3 /workspace/ros2_ws/test/run_vda_video_depth.py \
  --repo /workspace/jhyoon_external/Video-Depth-Anything \
  --input /workspace/ros2_ws/test/data/generated_can_motion_small_rotating_vda/can_motion_rgb.mp4 \
  --checkpoint /workspace/jhyoon_external/models/metric_video_depth_anything_vits.pth \
  --output /workspace/ros2_ws/test/data/generated_can_motion_small_rotating_vda/vda_depths.npz \
  --depth-dir /workspace/ros2_ws/test/data/generated_can_motion_small_rotating_vda/depth_mm \
  --visualization-video /workspace/ros2_ws/test/data/generated_can_motion_small_rotating_vda/vda_depth_visualization.mp4'
```

```bash
docker compose -f src/jhyoon_external/docker-compose.yml -f src/jhyoon_external/docker-compose.offline-gpu.yml run --rm perception \
  bash -lc 'python3 -m pip install --quiet --disable-pip-version-check git+https://github.com/ultralytics/CLIP.git ftfy regex && cd /workspace/jhyoon_external/models && \
  python3 /workspace/ros2_ws/test/track_pose_video.py \
  --rgb-video /workspace/ros2_ws/test/data/generated_can_motion_small_rotating_vda/can_motion_rgb.mp4 \
  --depth-npz /workspace/ros2_ws/test/data/generated_can_motion_small_rotating_vda/vda_depths.npz \
  --camera-json /workspace/ros2_ws/test/data/generated_can_vda_output/camera.json \
  --weights /workspace/jhyoon_external/models/yoloe-26m-seg.pt \
  --overlay-output /workspace/ros2_ws/test/data/generated_can_motion_small_rotating_vda/pose_tracking_overlay.mp4 \
  --json-output /workspace/ros2_ws/test/data/generated_can_motion_small_rotating_vda/pose_tracking.json'
```

현재 실행 결과는 19/20 pose frame 유효, detection ratio `0.95`, 추정 trajectory 길이 약 `0.251 m`입니다. 이 영상은 cutout을 축소·회전·이동시킨 synthetic motion이므로 tracking continuity 확인용이며, 실제 pose accuracy 평가는 ground-truth 카메라 영상과 aligned depth가 필요합니다.

### 50% 크기 Box pose 테스트

원통형 캔보다 PCA 방향이 명확한 직육면체 box를 선택해 현재 크기의 약 50%로 줄인 RGB를 생성했습니다.

- RGB: `test/data/generated_box_half_headcam_rgb.png`
- VDA depth: `test/data/generated_box_half_vda/depth_mm/000000.png`
- pose overlay: `test/data/generated_box_half_vda/pose_grasp_overlay.jpg`
- pose JSON: `test/data/generated_box_half_vda/pose.json`

검출 label은 `box`이며, 테스트 결과 PCA pose center는 `[0.0318, -0.0475, 1.008] m`, quaternion은 `[0.7068, 0.0226, -0.0226, 0.7068]`, orientation confidence는 `0.840`입니다. VDA metric depth의 크기 오차로 grasp width는 별도 reject됐지만, box mask와 자세 축 추정은 성공했습니다.

### 약 1m 거리 20초 Box pose tracking

50% box를 시작 이미지로 사용해 10fps·200-frame(약 20초) RGB 영상을 만들고 VDA depth 및 frame별 pose를 저장했습니다.

- RGB 영상: `test/data/generated_box_1m_20s_vda/box_motion_20s_rgb.mp4`
- VDA depth 영상: `test/data/generated_box_1m_20s_vda/vda_depth_visualization.mp4`
- raw VDA depth: `test/data/generated_box_1m_20s_vda/vda_depths.npz`
- pose overlay: `test/data/generated_box_1m_20s_vda/pose_tracking_20s_overlay.mp4`
- pose trajectory: `test/data/generated_box_1m_20s_vda/pose_tracking_20s.json`

추정된 유효 frame은 `176/200`(`88%`)이고, 유효 frame의 `z` 중앙값은 `1.016 m`(`0.980–1.061 m`)이었습니다. trajectory 길이는 약 `0.576 m`입니다. 마지막 회전 구간에서 YOLOE 검출이 일부 누락되므로 JSON의 `detected` 필드로 확인할 수 있습니다.

### 3D 일관 RGB-D Box 회전 검증

위 20초 영상은 물체 cutout을 영상 평면에서 회전시킨 tracking-continuity 테스트입니다. 실제 자세 변화 검증에는 `generate_box_3d_rgbd_video.py`를 사용합니다. 이 생성기는 cuboid의 3D yaw로 RGB를 투영하고 같은 geometry의 metric z-buffer depth를 함께 저장합니다.

```bash
docker compose -f src/jhyoon_external/docker-compose.yml -f src/jhyoon_external/docker-compose.offline-gpu.yml run --rm --no-deps perception \
  bash -lc 'cd /workspace/ros2_ws && \
  python3 /workspace/ros2_ws/test/generate_box_3d_rgbd_video.py \
  --rgb /workspace/ros2_ws/test/data/generated_box_half_headcam_rgb.png \
  --weights /workspace/jhyoon_external/models/yoloe-26m-seg.pt \
  --output /workspace/ros2_ws/test/data/generated_box_3d_20s/box_3d_motion_20s_rgb.mp4 \
  --depth-output /workspace/ros2_ws/test/data/generated_box_3d_20s/ground_truth_depths.npz \
  --depth-dir /workspace/ros2_ws/test/data/generated_box_3d_20s/depth_mm \
  --camera-output /workspace/ros2_ws/test/data/generated_box_3d_20s/camera.json \
  --metadata-output /workspace/ros2_ws/test/data/generated_box_3d_20s/motion_metadata.json \
  --mask-output /workspace/ros2_ws/test/data/generated_box_3d_20s/source_mask.png \
  --frames 200 --fps 10 --yaw-deg 60'
```

```bash
docker compose -f src/jhyoon_external/docker-compose.yml -f src/jhyoon_external/docker-compose.offline-gpu.yml run --rm --no-deps perception \
  bash -lc 'cd /workspace/ros2_ws && \
  python3 /workspace/ros2_ws/test/track_pose_video.py \
  --rgb-video /workspace/ros2_ws/test/data/generated_box_3d_20s/box_3d_motion_20s_rgb.mp4 \
  --depth-npz /workspace/ros2_ws/test/data/generated_box_3d_20s/ground_truth_depths.npz \
  --camera-json /workspace/ros2_ws/test/data/generated_box_3d_20s/camera.json \
  --weights /workspace/jhyoon_external/models/yoloe-26m-seg.pt --prompt box --confidence 0.02 \
  --overlay-output /workspace/ros2_ws/test/data/generated_box_3d_20s/pose_tracking_ground_truth_overlay.mp4 \
  --json-output /workspace/ros2_ws/test/data/generated_box_3d_20s/pose_tracking_ground_truth.json'
```

이 검증에서 YOLOE 검출은 `200/200`, 위치 trajectory는 약 `0.405 m`였습니다. 실제 cuboid yaw는 `60°`였지만 visible point cloud PCA의 시작–끝 orientation 변화는 약 `24.1°`였습니다. 따라서 PCA 결과는 정확한 6D object pose가 아니라 table-normal을 고정한 grasp orientation 근사값으로 해석해야 합니다. 정확한 yaw가 필요하면 CAD mesh를 이용한 FoundationPose/ICP 또는 별도의 model-based pose solver가 필요합니다.

같은 RGB 영상에 VDA depth를 넣은 결과는 위치 trajectory 약 `2.288 m`, orientation 변화 약 `6.0°`로 더 불안정했습니다. 따라서 VDA는 현재 환경에서 metric 위치와 회전 평가용 ground truth를 대신할 수 없고, 카메라 연결 전 파이프라인 smoke test 용도로만 사용해야 합니다.

### 방 안 테이블 ISO view 테스트

실제 head-camera에 가까운 장면을 위해 방, 바닥, 벽, 테이블, 박스가 함께 보이는 elevated ISO view 기준 이미지를 추가했습니다.

- 기준 RGB: `test/data/generated_box_room_iso_headcam_rgb.png`
- 20초 RGB 영상: `test/data/generated_box_room_iso_3d_20s/box_3d_motion_20s_rgb.mp4`
- VDA depth: `test/data/generated_box_room_iso_3d_20s/vda_depths.npz`
- pose overlay (`blue box` prompt): `test/data/generated_box_room_iso_3d_20s/pose_tracking_vda_blue_box_overlay.mp4`
- pose JSON (`blue box` prompt): `test/data/generated_box_room_iso_3d_20s/pose_tracking_vda_blue_box.json`

단순 `box` prompt에서는 confidence 평균 `0.053`, 검출률 `60%`였지만, `blue box` prompt에서는 confidence 평균 `0.713`(중앙값 `0.765`, 범위 `0.287–0.920`), 검출률 `100%`로 개선되었습니다. `blue box` 결과의 trajectory는 약 `1.728 m`이고, VDA-PCA orientation 변화는 `0.995°`입니다. 실제 캔 노드의 prompt는 `can`으로 유지하고, 물체별로 색상·형상을 포함한 구체적인 prompt를 사용하는 것이 좋습니다.

## FoundationPose 연결

FoundationPose는 YOLOE처럼 이미지에서 물체를 자동으로 찾는 검출기가 아닙니다. YOLOE가 만든 mask와 카메라 intrinsics, depth, 그리고 물체의 CAD mesh를 입력으로 받아 물체 좌표계의 6D pose를 계산합니다. 공식 구현의 model-based 경로를 연결했으며, label별 mesh를 지정하면 첫 프레임은 `register`, 이후 프레임은 `track_one`으로 처리합니다.

통합 Docker 이미지에는 FoundationPose의 CUDA 확장과 Python 런타임이 포함되어 있습니다. 다만 수 GB 단위의 공식 가중치, 예제 데이터, CAD mesh는 호스트에 준비해 bind mount해야 하며, `mycpp` 확장은 컨테이너 시작 시 필요할 때 자동 빌드됩니다. 공식 안내는 [NVlabs/FoundationPose README](https://github.com/NVlabs/FoundationPose)를 참고하세요.

호스트에서 외부 저장소를 bind mount할 위치를 준비합니다. 이미 저장소가 있다면 다시 clone하지 않아도 됩니다.

```bash
cd /home/irl/Desktop/IRLRoboticsArchitecture
git clone https://github.com/NVlabs/FoundationPose.git src/jhyoon_external/FoundationPose
```

FoundationPose 공식 가중치는 `src/jhyoon_external/FoundationPose/weights/`에 넣고, 사용할 CAD mesh는 `src/jhyoon_external/objects/`에 둡니다. 예를 들어 `src/jhyoon_external/objects/bottle/bottle.obj`가 있고 YOLOE 결과 label이 `bottle`인 경우 `src/perception/irl_object_perception/config/perception.yaml`을 다음처럼 바꿉니다.

```yaml
pose_backend: foundationpose
foundationpose_meshes: "bottle=/workspace/jhyoon_external/objects/bottle/bottle.obj"
```

여러 물체는 세미콜론으로 연결합니다.

```yaml
foundationpose_meshes: "bottle=/workspace/jhyoon_external/objects/bottle/bottle.obj;tool=/workspace/jhyoon_external/objects/tool/tool.obj"
foundationpose_mesh_scales: "bottle=0.001;tool=1.0"
```

`foundationpose_mesh_scales`는 label별 메시 좌표에 곱할 양수 배율이며, 생략하면 `1.0`입니다. 입력한 메시와 카메라 depth는 같은 단위여야 합니다. 현재 Can 설정은 실물 높이 135 mm 기준으로 `can=0.067861908`을 적용합니다.

실행 명령은 기존 YOLOE 노드와 같습니다.

```bash
docker compose -f src/jhyoon_external/docker-compose.yml up perception
```

FoundationPose 런타임이나 mesh가 준비되지 않은 상태에서 `pose_backend: auto`를 사용하면 해당 물체는 기존 depth-mask PCA 좌표계로 자동 처리됩니다. `object_poses`와 `object_<label>_<index>` TF 인터페이스는 두 경로에서 동일합니다.

## 통합 Docker에서 FoundationPose 사용

이제 ROS 2, YOLOE, FoundationPose가 모두 `irl-perception:jazzy` 하나의 이미지에 들어 있습니다. RTX 50 시리즈 호환을 위해 PyTorch 2.10 + CUDA 12.8을 사용합니다. `src/jhyoon_external/FoundationPose`는 호스트에서 bind mount되며, 컨테이너 시작 시 `mycpp` 확장이 필요할 때 자동 빌드됩니다.

먼저 공식 저장소, 가중치, mustard 예제 데이터가 다음 위치에 있어야 합니다.

```text
src/jhyoon_external/FoundationPose/
├── demo_data/mustard0/
├── weights/2023-10-28-18-33-37/
└── weights/2024-01-11-20-02-45/
```

통합 이미지를 빌드합니다.

```bash
docker compose -f src/jhyoon_external/docker-compose.yml build perception
```

ROS2 노드에서 YOLOE mask와 FoundationPose를 함께 사용하려면 [perception.yaml](/home/irl/Desktop/IRLRoboticsArchitecture/src/perception/irl_object_perception/config/perception.yaml)의 mesh 설정을 지정합니다.

```yaml
pose_backend: foundationpose
foundationpose_meshes: "bottle=/workspace/jhyoon_external/objects/bottle/bottle.obj"
```

그 다음 하나의 컨테이너로 ROS2 perception을 실행합니다.

```bash
xhost +si:localuser:root
docker compose -f src/jhyoon_external/docker-compose.yml up perception
```

FoundationPose를 사용할 수 없는 물체는 `pose_backend: auto` 설정에서 기존 PCA 좌표계로 자동 처리됩니다.

## 실시간 Can 자세 추정 직접 실행

텍스트 프롬프트 `soda can`으로 캔을 찾아 그 mask와 `Can1_textured.obj`를 FoundationPose에 전달하는 수동 실행 방법입니다. 이 테스트에는 text prompt를 지원하는 `yoloe-26m-seg.pt`와 Ultralytics 8.4.0 이상이 필요합니다. 처음 `set_classes()`를 호출할 때 text encoder(약 254 MB)가 내려받아질 수 있으므로 컨테이너에 네트워크 연결이 필요합니다. `/workspace/jhyoon_external/models`에서 실행하면 encoder가 bind mount 안에 저장되어 다음 실행에서 재사용됩니다. 캔을 카메라에 가깝고 크게 보이게 하세요. 다른 표현을 시험하려면 `--prompt "beverage can"`처럼 바꿀 수 있습니다.

호스트에서 GUI 권한을 열고, **터미널 1**에서 카메라 드라이버만 실행합니다. 컨테이너 Bash에 들어가면 아래 `ros2 launch`를 실행하세요.

```bash
# 호스트에서
xhost +si:localuser:root
docker compose -f src/jhyoon_external/docker-compose.yml run --rm -it perception bash

# 터미널 1의 컨테이너 Bash에서: RealSense만 실행
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=/ camera_name:=camera \
  enable_color:=true enable_depth:=true \
  align_depth.enable:=true enable_sync:=true \
  rgb_camera.color_profile:=640x480x30 \
  depth_module.depth_profile:=640x480x30 \
  pointcloud.enable:=false
```

카메라가 켜진 것을 확인한 뒤 **터미널 2**를 열어 별도 컨테이너에서 추론 스크립트를 실행합니다. `docker compose -f src/jhyoon_external/docker-compose.yml run`은 bash를 열 뿐 perception 노드를 자동으로 시작하지 않습니다.

```bash
# 터미널 2의 호스트에서
docker compose -f src/jhyoon_external/docker-compose.yml run --rm -it perception bash

# 터미널 2의 컨테이너 Bash에서: 필요할 때만 체크포인트 다운로드
if [ ! -f /workspace/jhyoon_external/models/yoloe-26m-seg.pt ]; then
  curl -L --fail https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-26m-seg.pt \
    -o /workspace/jhyoon_external/models/yoloe-26m-seg.pt
fi

python3 -c 'import ultralytics; print(ultralytics.__version__)'  # 8.4.0 이상이어야 함
cd /workspace/jhyoon_external/models  # text encoder를 bind mount에 캐시
python3 /workspace/ros2_ws/test/can_reference_live.py
# 필요하면 표현이나 threshold를 지정
# python3 /workspace/ros2_ws/test/can_reference_live.py --prompt "beverage can" --confidence 0.1
```

카메라 화면 앞에 캔 하나를 두면 YOLOE mask와 confidence, FoundationPose XYZ 및 X/Y/Z 축이 보입니다. 실시간 스크립트에는 실물 높이 135 mm 기준의 메시 배율 `0.067861908`이 적용됩니다. 끝낼 때는 터미널 2의 영상 창에서 `q`, 터미널 1에서는 `Ctrl+C`를 누르세요. 두 컨테이너는 host network를 공유하므로 같은 RealSense 토픽을 사용합니다. GUI 권한은 호스트에서 `xhost -si:localuser:root`로 되돌립니다. 스크립트는 [test/can_reference_live.py](test/can_reference_live.py)에 있습니다.

## 공식 `run_demo.py` 테스트

같은 통합 컨테이너에서 공식 mustard 데모도 실행할 수 있습니다.

```bash
xhost +si:localuser:root
docker compose -f src/jhyoon_external/docker-compose.yml run --rm perception \
  python3 /workspace/jhyoon_external/FoundationPose/run_demo.py \
  --debug 1 \
  --debug_dir /workspace/jhyoon_external/FoundationPose/debug_run
```

GUI 없이 실행하려면 `--debug 0`으로 바꿉니다. 결과 4×4 pose는 다음 위치에 저장됩니다.

```text
src/jhyoon_external/FoundationPose/debug_run/ob_in_cam/*.txt
```

생성된 pose 개수는 다음처럼 확인할 수 있습니다. mustard 예제에서는 737개가 생성되어야 합니다.

```bash
find src/jhyoon_external/FoundationPose/debug_run/ob_in_cam -maxdepth 1 -type f -name '*.txt' | wc -l
```

`--debug 2`를 사용하면 tracking overlay 이미지도 `debug_run/track_vis/`에 저장됩니다. 테스트가 끝나면 X11 권한을 되돌립니다.

```bash
xhost -si:localuser:root
```
