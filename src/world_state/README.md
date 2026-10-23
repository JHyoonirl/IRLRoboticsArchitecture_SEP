# World State 계약

World State는 task 이름이나 skill의 성공/실패 결과를 저장하는 buffer가
아니다. Sensing, estimation, perception, action feedback에서 들어온 현재
사실을 하나의 일관된 형태로 제공하고, task manager/BT와 skill이 그 사실을
읽어 precondition과 outcome을 평가하는 shared state representation이다.

## Skill과의 관계

Skill이 먼저 필요한 상태를 정의한다. 예를 들어 `Pick(tool_01)`은 다음과
같은 사실을 요구할 수 있다.

- `objects`에 `tool_01`이 보인다.
- 해당 object의 `pose_valid`가 true다.
- `robot.valid`가 true다.
- `gripper.valid`가 true이고 `occupied`가 false다.

이 조건식 자체와 성공/실패 판정은 BT에 둔다. World State에는
`preconditions_met`, `skill_succeeded` 같은 task-specific 필드를 넣지 않는다.

## 실행과 입출력

```bash
ros2 launch irl_bringup world_state.launch.py
ros2 topic echo /state/objects
```

설정 파일은 `irl_world_state/config/world_state.yaml`이다. 다른 파일은
`config_file:=/absolute/path.yaml`로 지정한다. 설정 변경 후 노드를 재시작한다.
`ros2 run irl_world_state world_state`도 동일한 기본 파라미터로 실행할 수 있다.

- 입력 `/perception/objects`: segmentation별 원본 pose, bbox, detector score
- 출력 `/state/objects`: 필터링한 `WorldObjectArray`, 기본 10 Hz
- 출력 `/state/world.objects`: 위와 동일한 추적 물체 목록
- `/perception/detections`, `/perception/object_poses`: 원본 진단 정보로 보존
- `/state/robot`, `/state/gripper`: 기본 1초 freshness로 수신

`world.valid`는 하나 이상의 최근 입력이 있다는 뜻이다. 모든 필드 또는
특정 행동의 실행 조건이 유효하다는 뜻이 아니다. 현재 robot estimator는
identity pose와 `system_status=mock`을 발행한다. 실제 gripper feedback이
없으면 `gripper.valid=false`이다.

## ID와 자세 필터

검출 배열 index를 ID로 사용하지 않는다. 같은 클래스의 물체들을 예측한 3D
위치와 관측 위치 사이의 비용으로 전역 할당한다. 20 cm 거리 gate와
Mahalanobis gate를 모두 적용하여 갑자기 튄 관측이 기존 물체를 옮기지 않게
한다. 깊이가 일시적으로 없을 때에는 같은 클래스의 2D bbox IoU로 연결한다.

위치와 속도는 6차원 constant-velocity Kalman filter로 추정한다. 위치 공분산을
함께 발행하며, 방향은 quaternion 보간으로 완화한다. PCA의 180도 축 부호
모호성은 직전 방향과 가까운 표현을 선택한다. 이것이 CAD 원점에 대한
정밀 자세나 deformable 물체의 유일한 방향을 보장하지는 않는다.

새 물체는 `tentative`이며 최소 3회 유효한 자세 관측과 0.4초가 지나면
`track_confirmed=true`가 된다. 관측이 잠깐 없어지면 `coasting` 상태로 예측하고
ID를 최대 2초 유지한다. 이후 삭제하며, 노드 수명 중 삭제된 ID는 재사용하지
않는다. 노드 재시작 시 ID는 초기화된다. 오래 가려졌거나 같은 클래스가
서로 교차하는 장면에서 완전한 재식별을 제공하는 appearance tracker는 아니다.

## 로봇 좌표계와 시간

기본 tracking frame은 `base_link`다. 관측 시각의 TF를 조회하여 위치와 방향을
변환한다. Estimation의 저장된 카메라 TF와 RealSense optical TF가 필요하다.
TF가 없으면 카메라 좌표를 base 좌표로 간주하지 않고 `missing_transform`으로
남겨 자세/action 유효성을 제한한다. 0.75초 이상 지연되거나 순서가 뒤집힌
메시지는 갱신에 사용하지 않는다. `observation_age_sec`에는 센서 timestamp와
수신 시각 사이의 지연도 포함한다.
위치는 측정 지연만큼 추정 속도로 전진 예측하고 그에 따른 공분산도 추가한다.
이는 일정 속도 가정의 보상이며, 오래된 측정을 최신 측정처럼 취급하지 않는다.

출력 object의 header는 필터 예측 시각과 tracking frame이다. Raw detection,
pose와 robot 상태는 각자 원본 header를 유지한다. Object bbox는 원본 카메라의
픽셀 좌표이며 3D tracking frame 좌표가 아니다.

확인된 유효한 track은 `base_link -> tracked_<object_id>` 동적 TF도 발행한다.
로봇은 `/state/objects`와 이 TF를 사용한다. Perception은 프레임별 배열 index를
사용하는 `object_*` TF를 발행하지 않는다. 원본 관측은 `/perception/objects`로
확인한다. TF buffer에 이미 등록된 과거 이름은 RViz를 재시작해 지울 수 있다.

## Confidence와 로봇이 사용할 필드

| 필드 | 의미 |
|---|---|
| `object_id` | 검출 순서와 독립적인 track ID |
| `pose`, `velocity` | tracking frame의 필터 위치/방향, m/s 속도 |
| `position_covariance` | 3x3 위치 공분산, row-major, m² |
| `detection_confidence` | 마지막 원본 YOLOE score |
| `confidence` | 시간적으로 감쇠시킨 operational trust, 0~0.99 |
| `visible` | 최근 입력 frame에서 관측되었고 관측 시점도 최근인지 |
| `pose_valid` | 최근의 3D 자세와 허용 범위 내 위치 불확실성이 있는지 |
| `track_confirmed` | 관측 횟수와 나이 조건을 만족한 track인지 |
| `tracking_status` | tentative / tracking / coasting |
| `observation_age_sec` | 마지막 실제 관측 이후 시간 |
| `observation_count` | 누적 matching 관측 횟수 |
| `actionable` | 일반적인 관측 품질 기준을 만족하는지 |

`confidence`는 YOLOE score를 그대로 복사하지 않는다. 원본 score의 시간 EMA,
관측 누적 성숙도, 위치 공분산, 위치/방향의 관측 일관성, 관측 최신성을 곱해
목표 점수를 만들고 1차 저역 필터로 추종한다. 상승 time constant는 1.0초,
하강은 0.7초이다. 미검출/깊이 손실 시 목표는 0으로 내려가지만 출력 confidence는
즉시 0이 되지 않는다. 실험 데이터로 calibration한 성공 확률은 아니며, 낮은
YOLOE score가 단순히 반복 관측되었다는 이유만으로 높은 신뢰도가 되지 않는다.

`actionable`은 확인된 track, 최근의 유효한 실제 자세 관측, 위치 불확실성,
방향 보정 오차, confidence를 함께 확인한다. Confidence 0.60 이상에서 켜지고
0.40 미만에서 꺼지는 hysteresis가 있다. 하지만 미검출, 깊이 손실, 0.3초 초과
관측 지연에는 confidence가 남아 있어도 즉시 꺼진다. 이는 관측 품질 flag이며
BT의 grasp 가능성, robot/gripper 상태 또는 skill precondition 판단을 대체하지
않는다. `pose_valid=false`인 pose 숫자는 실행 목표로 사용하지 않는다.

주요 조정값은 다음과 같다.

- `tracker_association_distance`: 같은 물체로 연결할 최대 이동 거리, m
- `tracker_measurement_std`: 측정 위치 잡음 가정, m
- `tracker_acceleration_std`: 움직임에 대한 process noise
- `tracker_retention`: 미관측 ID 보존 시간, s
- `tracker_confidence_rise_tau`, `tracker_confidence_fall_tau`: confidence damping, s
- `tracker_ready_on`, `tracker_ready_off`: 품질 flag의 hysteresis 경계

PCA/검출 오차와 실제 로봇 작업 조건에 맞춰 검증하고 조정해야 한다.

Perception은 기본 `sensor_qos_depth=1`, `sync_queue_size=2`로 최신 RGB-D
프레임을 우선 처리한다. 추론이 느릴 때 과거 프레임을 순서대로 따라가는
지연을 줄이는 설정이다. `inference_imgsz`와 grasp/PCA 처리 시간 자체가
길면 `observation_age_sec`가 높아질 수 있으며, 최신성 조건을 완화하기보다
처리 지연을 먼저 확인한다.
