# YOLOE image prompts

`rigid_object_*.jpg`는 같은 물체의 서로 다른 시점으로
사용합니다. 출력 class 이름은 `rigid_object`입니다.

`config/perception.yaml`에서 설정합니다.

```yaml
image_prompt_dir: /workspace/ros2_ws/src/perception/irl_object_perception/image_prompt
image_prompt_label: rigid_object
prompt_mode: hybrid
segmentation_prompt: black glove
inference_imgsz: 960
visualization_target: rigid_object
grasp_target: rigid_object
```

노드 시작 시 폴더의 JPG/JPEG/PNG를 정렬해서 읽고, 각 이미지 전체를 대상
bounding box로 사용해 visual prompt embedding을 추출합니다. 각 시점의
embedding을 평균한 뒤 정규화하여 한 클래스에 등록합니다. 이후 카메라
프레임은 등록한 visual embedding으로 추론하므로 프레임마다 참조 이미지를
다시 인코딩하지 않습니다. `image` 모드에서는 텍스트 인코더를 사용하지 않습니다.

## 텍스트와 이미지를 함께 사용

`prompt_mode`로 `text`, `image`, `hybrid`, `auto`를 선택합니다. `auto`는
텍스트와 이미지 경로가 모두 있으면 hybrid, 이미지 경로만 있으면 image,
그 외에는 text 모드를 선택합니다.

`hybrid`는 이미지 대상과 텍스트 대상을 서로 다른 클래스로 등록합니다.
이미지 특징은 `rigid_object`, 텍스트 특징은 `black glove`에 대응합니다.
두 대상의 특징을 가중 평균하지 않습니다. YOLOE의 embedding API를 이용해
클래스 축으로 연결하고 한 번의 추론에서 둘을 찾습니다. 텍스트 인코더는
models 폴더의 기존 파일을 재사용합니다.

텍스트 대상을 여러 개 찾으려면 `segmentation_prompt: "black glove, cup"`처럼
쉼표로 구분합니다. 이미지 클래스와 텍스트 클래스 이름은 달라야 합니다.
grasp/visualization target은 검출 클래스들 중 원하는 이름으로 지정합니다.
현재 기본 grasp/visualization target은 `rigid_object`이며 detection 토픽에는
두 클래스 모두 포함됩니다.

출력 이름과 모델에 입력하는 설명은 `text_prompt_aliases`로 분리할 수 있습니다.

```yaml
segmentation_prompt: orange glove
text_prompt_aliases: '{"orange glove": "orange rubber glove"}'
```

이 설정은 `orange rubber glove`의 text embedding을 사용하고, 출력 class는
`orange glove`로 유지합니다. Confidence 숫자를 재조정하지 않으며 YOLOE의
원본 추론 점수를 그대로 제공합니다. 실제 카메라 프레임에서 설명별 위치와
점수를 함께 비교하여 선택해야 합니다. 설정 변경 후 perception을 재시작합니다.

## 작은 물체

`inference_imgsz`는 카메라 프레임 추론 해상도이며 기본값은 960입니다.
카메라 촬영 해상도와 별개입니다. 640, 960, 1280처럼 32의 배수로 설정할 수
있으며 높은 값은 연산량과 메모리 사용을 늘립니다. 원본 영상에 세부 정보가
없으면 추론 해상도만 높여도 해결되지 않습니다. 실제 사용 거리와 조명에서
촬영한 물체 crop을 참조 이미지로 추가하거나 카메라 시야를 조정할 수 있습니다.
설정 변경 후 perception 노드를 재시작합니다.

새 이미지는 배경을 최소화하여 물체가 화면을 채우도록 잘라 넣습니다.
이 폴더의 모든 이미지는 같은 클래스의 참조로 합쳐집니다. 여러 종류의
물체를 별도 클래스로 등록하는 폴더 구조는 현재 지원하지 않습니다.
이미지나 라벨을 변경한 후 perception 노드를 재시작합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash
cd /workspace/jhyoon_external/models
ros2 launch irl_bringup perception.launch.py camera_mode:=rgbd
```

출력은 `/perception/detections`와 `/perception/object_poses`이며 World State가
실행 중이면 `/state/objects`에도 반영됩니다. 텍스트 프롬프트만 사용하려면
`prompt_mode: text`와 원하는 `segmentation_prompt`를 설정하고 출력 target도
같은 이름으로 맞춥니다. 이미지 프롬프트만 사용하려면 `prompt_mode: image`로
설정합니다.

## 모든 물체의 자세

이미지/텍스트로 검출한 모든 instance에 대해 depth-PCA 또는 FoundationPose로
자세를 추정합니다. Grasp target 선택과 별개이며, 화면에도 유효한 모든 물체의
자세 축을 표시합니다. PCA는 보이는 depth 점들의 중심과 주축으로 정의한
좌표계이며 물체 CAD 원점에 대한 정밀한 자세와는 다릅니다.

`/perception/objects` (`WorldObjectArray`)에는 segmentation된 모든 물체를
ID, class, confidence, pose, pose_valid, pose_source, pose_status와 함께 발행합니다.
깊이가 부족한 물체를 배열에서 제거하거나 다른 물체의 pose를 할당하지 않습니다.
추정할 수 없으면 `pose_valid=false`로 표시합니다. Perception은 원본 물체
TF를 발행하지 않으며, World State만 확인된 유효한 track의 `tracked_*` TF를
발행합니다.
World State는 이 관측을 추적하여 `/state/objects`와 `/state/world.objects`로
안정된 ID, 필터 pose와 operational confidence를 제공합니다. 원본 detector
점수는 `detection_confidence`이며, raw `/perception/objects`의 confidence와
로봇용 `/state/objects`의 confidence는 의미가 다릅니다.
기존 `/perception/object_poses`는 유효한 pose만 포함하는 보조 출력입니다.
