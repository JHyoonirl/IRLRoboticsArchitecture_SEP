# IRL Camera Calibration

Estimate a fixed camera's pose in `base_link` from a stationary ChArUco board.
The board specification and its measured center pose are configured separately
from the calibration node.

## Configuration

Edit `config/charuco_board.yaml` to match the printed board. `squares_x` and
`squares_y` count full board squares, and square and marker lengths are in
metres. Measure the printed square after printing; printer scaling affects the
translation estimate directly. The current board is configured as 5 x 5
squares, with 20 mm squares and 15 mm markers using `DICT_5X5_100`.

Edit `board_pose.translation_mm` in `config/calibration.yaml` to set the board
center in the parent frame. The current example is `[310.5, -12.5, 8.0]` mm in
`base_link`. Its orientation maps `base_link +X` to marker `-Y`, `base_link +Y`
to marker `-X`, and `base_link +Z` to marker `-Z`. The ChArUco coordinate
origin is shifted to the center of the board before solving its pose.

## Run with Docker

Run these commands on the host from the repository root. The first command
allows the container's root user to open RViz in the current desktop session;
the Compose service then starts the RealSense camera with the rest of the
architecture.

```bash
xhost +si:localuser:root
docker compose -f src/jhyoon_external/docker-compose.yml up -d --force-recreate perception
```

Open a second host terminal, enter the running container, and build the
calibration package into the persistent workspace install volume:

```bash
docker compose -f src/jhyoon_external/docker-compose.yml exec perception bash
```

Inside that shell, run:

```bash
cd /workspace/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src --packages-select irl_camera_calibration
source install/setup.bash
ros2 topic list | grep '/camera/color'
```

The camera topics should include `/camera/color/image_raw` and
`/camera/color/camera_info`. Leave this shell open. In a third host terminal,
open another shell in the same container:

```bash
docker compose -f src/jhyoon_external/docker-compose.yml exec perception bash
```

Then source the workspace and start calibration with RViz:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash
ros2 launch irl_camera_calibration calibrate.launch.py start_rviz:=true
```

Use another host terminal to follow the camera and architecture logs if needed:

```bash
docker compose -f src/jhyoon_external/docker-compose.yml logs -f perception
```

The launch starts RViz with `base_link` as the fixed frame, showing the camera
image, TF tree, and grid. Keep the camera and board still while the node
collects 30 accepted observations. It rejects frames with too few corners or
excessive reprojection error, then filters inconsistent poses. A successful
estimate is written to `config/camera_extrinsics.yaml` and published as static
transforms from `base_link` to `charuco_board` and `camera_link`. The camera's
existing optical-frame transform is used to convert from the ChArUco pose to
`camera_link`. With the Compose source mount and symlink install, the result is
available on the host at
`src/sensing/irl_camera_calibration/config/camera_extrinsics.yaml`.

When finished, stop the service and revoke the temporary X11 access from the
host:

```bash
docker compose -f src/jhyoon_external/docker-compose.yml stop perception
xhost -si:localuser:root
```

Override topics, board pose, board dimensions, sample thresholds, and output
path through the two YAML files. A valid camera-to-board observation alone
cannot locate the camera in `base_link`; the board center pose in
`base_link` must be measured and entered accurately.

## Visualize the ChArUco axes

With the camera running, start the axes viewer in a separate container shell:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash
ros2 launch irl_camera_calibration view_charuco_axes.launch.py
```

RViz shows the live color image with the detected board axes drawn at its
center: `+X` red, `+Y` green, and `+Z` blue. The annotated image is also
published on `/camera/calibration/charuco_axes/image`. In an RViz window that
already displays the unmodified camera image, change the Image display's
`Image Topic` from `/camera/color/image_raw` to
`/camera/calibration/charuco_axes/image`; the overlay is a separate topic. Set
`axes_length_m` to change the displayed axis length, for example
`axes_length_m:=0.06`.
