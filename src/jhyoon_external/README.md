# jhyoon_external

[카메라 → Calibration → Perception → World State 단계별 실행](MANUAL_RUN.md)

Jihyun의 sensing/perception 구현이 의존하는 외부 코드와 실행 자산을 관리하는
독립 Git 경계입니다.

```text
jhyoon_external/
├── docker-compose.yml       # submodule 실행 진입점
├── docker-compose.offline.yml
├── docker-compose.offline-gpu.yml
├── FoundationPose/       # 별도 upstream Git 저장소를 유지
├── models/               # YOLOE/MobileCLIP 등 실행 가중치
├── objects/              # FoundationPose CAD mesh
├── requirements.txt      # Python runtime dependencies
└── docker/
    ├── Dockerfile
    ├── constraints.txt
    └── entrypoint.sh
```

`models/`와 `objects/`의 대용량 파일은 Git에 직접 넣지 않고 별도 배포/아티팩트로
공급합니다. Docker Compose는 이 디렉터리 전체를 `/workspace/jhyoon_external`로
마운트합니다.

이 submodule의 Docker는 부모 workspace의 `src/`와 `test/`를 함께 사용합니다. 따라서
부모 저장소의 `src/jhyoon_external` 위치에서 다음처럼 실행합니다.

부모 저장소에 연결할 때는 다음 위치를 submodule 경계로 사용합니다.

```bash
git submodule add <JHYOON_EXTERNAL_REMOTE_URL> src/jhyoon_external
git submodule update --init --recursive
```

```bash
cd /home/irl/Desktop/IRLRoboticsArchitecture/src/jhyoon_external
docker compose build perception
bash docker/compose-camera.sh up --force-recreate perception
```

카메라의 `/dev/videoN`, `/dev/mediaN` 번호는 호스트 및 USB 연결에 따라 달라질 수
있습니다. `compose-camera.sh`는 현재 존재하는 장치만 매핑하므로 고정 목록의
`/dev/video8` 등이 없어서 컨테이너 시작이 실패하는 문제를 피합니다.
장치를 재연결한 뒤에는 위 명령을 다시 실행해 매핑을 갱신합니다.
이 스크립트는 Docker daemon이 실행 중인 호스트에서 사용합니다.

Perception launch는 호스트에 마운트된 `models/`에서 실행합니다.
MobileCLIP의 `mobileclip2_b.ts` 같은 보조 가중치도 이 디렉터리에 저장되어
`--force-recreate` 이후 재사용됩니다. `YOLO_CONFIG_DIR`는 설정 파일 저장
위치이며, 작업 디렉터리에 다운로드되는 가중치를 보존하지는 않습니다.

offline 실행은 다음 override를 추가합니다.

```bash
docker compose -f docker-compose.yml -f docker-compose.offline.yml up perception
```

카메라 구성은 부모 workspace의 `src/sensing/irl_sensor`가 소유합니다.
Compose는 `USE_REALSENSE`와 `CAMERA_MODE`를 로컬 launch로 전달합니다.

```bash
USE_REALSENSE=true CAMERA_MODE=rgbd docker compose up perception
USE_REALSENSE=true CAMERA_MODE=rgb docker compose up perception
USE_REALSENSE=false CAMERA_MODE=rgb docker compose up perception
```

`docker-compose.offline.yml`과 `docker-compose.offline-gpu.yml`은 자동으로
`USE_REALSENSE=false`를 설정합니다. 따라서 Docker는 카메라 모드와 토픽
계약을 정의하지 않고, ROS source를 mount하고 외부 driver/runtime 의존성만
제공합니다.
