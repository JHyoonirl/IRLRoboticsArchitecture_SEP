# irl_visualization

World State와 perception 결과를 Foxglove Studio에서 바로 확인하기 위한
로컬 ROS 2 visualization package입니다.

## 실행

전체 architecture가 이미 실행 중인 경우 별도 터미널에서 실행합니다.

```bash
ros2 launch irl_visualization visualization.launch.py
```

Foxglove Studio에서 `ws://localhost:8765`로 연결합니다.

Docker에서는 image를 다시 빌드한 뒤 다음처럼 실행할 수 있습니다.

```bash
docker compose build perception
docker compose run --rm perception \
  ros2 launch irl_visualization visualization.launch.py
```

## Foxglove 패널 구성

- 3D panel: `/visualization/world_state_markers`
- Image panel: `/perception/debug_image`
- RGB image: `/sensing/camera/color/image_raw`
- Raw Messages: `/state/world`
- Raw Messages: `/visualization/world_state_summary`
- Diagnostics/Raw Messages: `/visualization/world_state_diagnostics`
- 3D panel의 fixed frame: `map` 또는 현재 robot state frame

3D marker에는 다음 정보가 표시됩니다.

- robot pose 방향 화살표
- object 위치 구체
- object X/Y/Z pose 축
- object ID, class, confidence, pose validity
- confidence에 따른 색상

`/perception/debug_image`에는 기존의 segmentation mask, confidence, XYZ,
pose 축이 함께 표시됩니다. Foxglove Bridge는 ROS 2의 기존 토픽과 이
visualization 토픽을 모두 전달하므로 개발 중에는 raw World State와 가공된
marker를 동시에 비교할 수 있습니다.
