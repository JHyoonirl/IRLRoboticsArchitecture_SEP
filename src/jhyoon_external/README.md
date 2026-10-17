# jhyoon_external

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
docker compose up --force-recreate perception
```

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
