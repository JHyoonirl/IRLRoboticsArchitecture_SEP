# 단계별 컨테이너 실행

호스트 저장소 루트에서 실행합니다. `models/`는 컨테이너의
`/workspace/jhyoon_external/models`로 마운트됩니다. MobileCLIP 파일은
`models/mobileclip2_b.ts`에 보존합니다.

## 컨테이너 준비 (호스트)

기존 perception 컨테이너를 수동 실행 모드로 재생성하므로 기존 노드는 종료됩니다.

```bash
# RViz/OpenCV 창이 필요할 때
xhost +si:localuser:root

bash src/jhyoon_external/docker/compose-camera.sh \
  -f "$PWD/src/jhyoon_external/docker-compose.manual.yml" \
  up -d --force-recreate perception

docker compose -f src/jhyoon_external/docker-compose.yml exec perception bash
```

처음 한 번 컨테이너 bash에서 빌드합니다. 다른 단계 실행 전에 완료합니다.

```bash
cd /workspace/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src
source install/setup.bash
```

## 각 단계는 별도 터미널에서 실행

각 호스트 터미널에서 같은 컨테이너에 들어갑니다.

```bash
docker compose -f src/jhyoon_external/docker-compose.yml exec perception bash
```

새 컨테이너 bash마다 아래 환경을 적용합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash
```

### 1. 카메라와 전처리

```bash
ros2 launch irl_bringup sensing.launch.py use_realsense:=true camera_mode:=rgbd
```

카메라는 `/camera/...`, 전처리는 `/sensing/camera/...`로 발행합니다.
카메라만 별도로 실행하려면 sensing launch 대신 아래 두 명령을 각각의
컨테이너 bash에서 실행합니다. 두 방식을 동시에 실행하지 않습니다.

```bash
ros2 launch irl_sensor camera.launch.py use_realsense:=true camera_mode:=rgbd
```

```bash
ros2 run irl_sensor_prepro sensor_preprocessor
```

### 2. Calibration (카메라 위치가 바뀐 경우에만)

인쇄 보드 규격과 실제 보드 중심 위치를 먼저 설정합니다.

- `/workspace/ros2_ws/src/sensing/irl_camera_calibration/config/charuco_board.yaml`
- `/workspace/ros2_ws/src/sensing/irl_camera_calibration/config/calibration.yaml`

```bash
ros2 launch irl_camera_calibration calibrate.launch.py \
  config_file:=/workspace/ros2_ws/src/sensing/irl_camera_calibration/config/calibration.yaml \
  start_rviz:=true
```

카메라와 보드를 고정하고 수집 완료를 기다립니다. 출력은 같은 config 폴더의
`camera_extrinsics.yaml`이며 호스트에도 보존됩니다. 저장 후 calibration을
종료하고 estimation을 실행합니다. 동일한 카메라 TF를 두 노드에서 동시에
발행하지 않습니다. 창 없이 실행하려면 `start_rviz:=false`를 사용합니다.
이 과정은 고정 카메라의 외부 파라미터 추정이며 카메라 내부 파라미터는
RealSense의 CameraInfo를 사용합니다.

### 3. Perception

```bash
cd /workspace/jhyoon_external/models
ros2 launch irl_bringup perception.launch.py camera_mode:=rgbd
```

`models/`에서 실행해야 MobileCLIP 다운로드 파일을 재사용합니다.
입력은 `/sensing/camera/...`, 출력은 `/perception/detections`,
`/perception/object_poses`입니다. 대상은
`/workspace/ros2_ws/src/perception/irl_object_perception/config/perception.yaml`의
`segmentation_prompt`, `grasp_target`, `visualization_target`에서 설정합니다.
GUI를 쓰지 않는 경우 같은 파일의 `show_visualization: false`를 설정합니다.

### 4. Estimation: 저장된 카메라 TF와 로봇 상태 (별도 bash)

```bash
ros2 launch irl_bringup estimation.launch.py \
  camera_extrinsics_file:=/workspace/ros2_ws/src/sensing/irl_camera_calibration/config/camera_extrinsics.yaml
```

일상 실행에서는 calibration 단계를 생략하고 이 명령으로 기존 카메라 위치를
적용합니다. `camera_tf` 노드가 YAML을 시작 시 읽어 `base_link -> camera_link`를
`/tf_static`으로 발행합니다. 파일이 바뀌면 estimation을 재시작합니다.
RealSense가 제공하는 `camera_link -> camera_color_optical_frame` TF와 연결됩니다.
카메라 TF만 필요하면 `use_mock_robot_state:=false`를 추가하거나
`ros2 run irl_robot_estimation camera_tf`를 사용합니다.

로봇 상태는 여전히 mock 테스트 입력입니다. 실제 로봇 상태가 있는 경우
`use_mock_robot_state:=false`를 사용합니다. 새 calibration 도중에는 estimation의
카메라 TF 발행을 중지하거나 `publish_camera_tf:=false`로 실행합니다.

### 5. World State (별도 bash)

```bash
ros2 launch irl_bringup world_state.launch.py
```

현재 `WorldState`는 `robot`, `objects`, `gripper`, 원본 `detections`,
`object_poses`, `valid`를 제공합니다. 기본 출력은 `/state/world` (10 Hz)이며
실제 gripper feedback이 없으면 gripper는 unknown 상태입니다. 물체는
칼만 필터로 추적하여 검출 순서가 바뀌어도 ID를 유지합니다. 잠깐 사라진
track은 기본 2초 보존하고 confidence를 서서히 낮춥니다.

Estimation의 카메라 TF를 이용해 World State의 물체 위치/방향은 `base_link`로
변환됩니다. 로봇은 `/state/objects`의 추적 ID, pose, 위치 공분산과
`actionable` 관측 품질 flag를 참고합니다. 원본 YOLOE score는
`detection_confidence`, 가공된 신뢰도는 `confidence`입니다.
TF는 `tracked_<object_id>` 이름으로 발행됩니다.

### 6. 확인 (별도 bash)

```bash
ros2 node list
ros2 topic hz /camera/color/image_raw
# Ctrl+C 후 다음 확인
ros2 topic hz /sensing/camera/aligned_depth_to_color/image_raw
# Ctrl+C 후 다음 확인
ros2 topic echo /perception/object_poses --once
ros2 topic echo /perception/objects --once
ros2 topic echo /state/objects --once
ros2 interface show irl_interfaces/msg/WorldState
ros2 interface show irl_interfaces/msg/WorldObject
ros2 topic echo /state/world --once
ros2 run tf2_ros tf2_echo base_link camera_link
```

## 종료 (호스트)

```bash
docker compose -f src/jhyoon_external/docker-compose.yml stop perception
xhost -si:localuser:root
```

전체 자동 실행으로 되돌리려면 manual override 없이 실행합니다.

```bash
bash src/jhyoon_external/docker/compose-camera.sh up --force-recreate perception
```
