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

## 현재 mock 계약

- `robot`: 현재 로봇 pose, twist, joint state
- `objects`: detection과 pose를 index 기준으로 결합한 환경 object 목록
- `gripper`: gripper 점유 여부와 opening width. 실제 feedback이 없으면
  `valid=false`인 unknown 상태다.
- `detections`, `object_poses`: 기존 perception 결과를 보존한 raw view

현재 object ID는 tracker가 연결되기 전까지 `class_index` 형식의 frame-local
ID다. 장기적인 skill 실행에는 perception tracker 또는 object memory가
안정적인 ID를 제공해야 한다.
