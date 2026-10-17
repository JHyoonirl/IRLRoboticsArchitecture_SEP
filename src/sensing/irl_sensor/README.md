# irl_sensor

프로젝트가 소유하는 카메라 입력 경계입니다. RealSense의 vendor driver를
복제하지 않고, 이 패키지의 `camera.launch.py`가 RGB/RGB-D 모드와 driver
사용 여부를 결정합니다. Docker는 `realsense2_camera` 실행 의존성만 제공하고,
모드/토픽 계약/launch 구성은 workspace의 이 패키지에서 관리합니다.

```bash
ros2 launch irl_sensor camera.launch.py use_realsense:=true camera_mode:=rgbd
ros2 launch irl_sensor camera.launch.py use_realsense:=true camera_mode:=rgb
ros2 launch irl_sensor camera.launch.py use_realsense:=false camera_mode:=rgbd
```

`use_realsense:=false`이면 카메라 driver를 실행하지 않고, `irl_sensor_prepro`
와 perception은 외부에서 공급되는 입력 토픽을 기다립니다.
