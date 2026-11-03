# bhl_hand_sdk

DexHand 양손(손마다 ESP32 1개, ES3352 서보 15개)을 팔과 같은 CAN 버스(`can0`, 1 Mbps)에서 제어하는 패키지입니다. `bhl_arm_sdk`와 같은 구조로, 순수 파이썬 코어(`controller.py`, ROS 없이 테스트)와 얇은 ROS 2 노드(`node.py`)로 나뉩니다.

핵심은 **명령마다 손이 받았는지를 돌려주는 것**입니다. v2 펌웨어(`firmware/PROTOCOL_v2.md`)를 쓰면, 50 Hz로 보내는 자세 세트마다 `APPLIED`, `NO_REPLY`, `REJECT_*` 중 하나가 돌아옵니다. v1 펌웨어(현재 손에 올라간 것)도 그대로 쓸 수 있고, 이때 결과는 `SENT`(확인 불가)입니다.

| 결과 | 뜻 |
|---|---|
| `APPLIED` | 손이 5개 프레임을 모두 받고 검사를 통과해 PWM을 기록함 |
| `REJECT_RANGE`, `REJECT_DEADLINE`, `REJECT_CONFLICT`, `REJECT_FORMAT` | 손이 받았지만 거부함(사유 포함, RANGE는 채널 번호) |
| `REJECT_INHIBITED` | 손이 HOLD/OFF 상태라 무시함 |
| `OUTPUT_FAIL` | PWM 기록 실패. 손이 출력을 끄고 래치함 |
| `NO_REPLY` | `reply_timeout` 안에 응답 없음: 프레임 손실, 손 꺼짐 또는 분리, 응답 손실. 다음 응답의 적용 횟수로 응답 손실(`acks_lost`)을 구분함 |
| `SENT` | v1 펌웨어: 어댑터에 넣었다는 것까지만 앎 |

어떤 결과도 손가락의 실제 위치를 뜻하지 않습니다. 서보에 피드백이 없으므로 `command_us`는 항상 명령값이고, 이 패키지는 손 `/joint_states`를 발행하지 않습니다.

## 실행

```bash
colcon build --symlink-install --packages-select bhl_hand_sdk
ros2 launch bhl_bringup robot.launch.py hands:=right                  # 팔 + 오른손, dry run
ros2 launch bhl_bringup robot.launch.py hands:=both dry_run:=false    # 실물
ros2 launch bhl_hand_sdk hand_sdk.launch.py hands:=right              # 손만

ros2 service call /hand_right/enable std_srvs/srv/SetBool "{data: true}"
ros2 param set /hand_right grip power          # power | pinch | point | open (config/dexhand.yaml)
ros2 param set /hand_right grip_level 0.6      # 0 펼침 .. 1 지정한 grip
ros2 topic echo /hand_right/delivery           # 세트마다 {"sequence", "result", "latency_ms", ...}
ros2 service call /hand_right/hold std_srvs/srv/Trigger    # success = CONTROL_ACK를 받음
```

| 이름 | 형식 | 내용 |
|---|---|---|
| `~/enable` | `SetBool` | true: PING → START(이미 PWM 출력 중이면 RESUME) → 첫 세트 = 현재 목표. false: HOLD |
| `~/hold`, `~/resume`, `~/off`, `~/estop`, `~/reset_estop`, `~/ping`, `~/open` | `Trigger` | `success`는 손이 확인했는지(v2), `message`는 돌아온 결과와 지연 |
| `~/target` | `Float64MultiArray` | 15개 당김 비율 0..1 (0 = 풀림 µs, 1 = 당김 µs) |
| `~/target_us` | `UInt16MultiArray` | 15개 펄스폭 µs (보정 범위 안만) |
| `~/delivery` | `String` JSON | 세트마다 전달 결과 |
| `~/command_us` | `UInt16MultiArray` | 명령한 펄스폭 (측정값 아님) |
| `~/status` | `String` JSON, 10 Hz | 상태, 손이 보낸 플래그, 적용 횟수, 통계 |
| `/diagnostics` | `DiagnosticArray`, 1 Hz | OK / WARN / ERROR |

채널 순서는 dexhand_control의 CAN 순서입니다: 소지 위, 아래, 약지 위, 아래, 중지 위, 아래, 검지 위, 아래, 검지 끝, 중지 끝, 약지 끝, 소지 끝, 엄지 오른쪽 당김, 엄지 왼쪽 당김, 엄지 끝. 당김 비율은 서보 행정의 비율이지 관절각이 아닙니다.

## 벤치 도구 (ROS 없이)

```bash
ros2 run bhl_hand_sdk hand_cli right ping                   # PONG: protocol 2, profile c7b282e4 (matches config)
ros2 run bhl_hand_sdk hand_cli right grip pinch --level 0.5 --hold-after
ros2 run bhl_hand_sdk hand_cli right grip power --protocol 1     # v1 펌웨어
ros2 run bhl_hand_sdk hand_cli right listen --seconds 3          # 하트비트 확인
```

`hand_controller`가 같은 손을 쓰고 있을 때는 실행하지 않습니다(손 하나에 송신자는 하나).

## 안전 장치

- enable 전에 PING을 보냅니다. 응답이 없거나 프로토콜이 다르거나 보정 프로필 태그가 다르면(왼손 펌웨어를 오른손에 올린 경우 등) 거부합니다.
- 손이 이미 PWM을 출력 중이면 START 대신 RESUME을 보냅니다. START는 PWM을 잠깐 끊어서 쥔 물체를 놓칩니다.
- `max_missed`(기본 10 = 0.2 s) 동안 APPLIED가 없으면 `LOST`로 송신을 멈춥니다. 손은 펌웨어 워치독으로 마지막 PWM을 유지합니다.
- 우리가 보내지 않은 HOLD/OFF로 손이 목표를 거부하면 `HELD_ELSEWHERE`로 멈춥니다.
- 엄지 좌우 당김(13, 14)처럼 서로 반대로 당기는 힘줄 쌍은 비율 합이 `max_pair_sum`(1.0)을 넘으면 보내기 전에 거부합니다. 다른 길항 쌍은 실측 후 `antagonist_pairs`에 추가해야 합니다.
- `/arm_controller/status`의 E-STOP을 따라 손도 `estop_action`(기본 hold: 쥔 채 유지, off: PWM 중단)을 실행합니다. 팔의 E-STOP 서비스가 손에 직접 전달되지 않던 공백을 메우는 장치입니다.
- 노드 종료 시 HOLD를 보냅니다.
- `dry_run`(기본)은 CAN을 열지 않고 모든 세트를 APPLIED로 처리합니다.

## 설정 (`config/dexhand.yaml`)

- `hands.<left|right>`: `node`(60/61), `open_us`, `close_us`(dexhand_control 프로필의 풀림/당김), `protocol`(1/2), `profile_tag`(BoardConfig.h `kProfileSha256` 앞 8자리)
- `control`: `control_rate_hz` 50, `max_speed` 2.0 행정/s, `reply_timeout` 5 ms(늦게 온 APPLIED는 `late`로 세고 실패에서 뺌), `max_missed` 10, `estop_action`, `antagonist_pairs`, `max_pair_sum`
- `grips`: 이름 있는 자세(당김 비율). 실제 손에서 확인해야 하는 시작값입니다.

## 펌웨어

| 폴더 | 내용 |
|---|---|
| `firmware/DexHandLeft/` | **왼손 보드**에 올릴 v2 스케치 (노드 60, 프로필 `c3c778d2`) |
| `firmware/DexHandRight/` | **오른손 보드**에 올릴 v2 스케치 (노드 61, 프로필 `c7b282e4`) |
| `firmware/UPLOAD_GUIDE.md` | 어느 보드에 무엇을 올리는지, 업로드 방법, PING으로 확인, 보정값 변경 시 순서 |
| `firmware/PROTOCOL_v2.md` | 손 → PC 응답(STATUS, PONG) 형식 |
| `firmware/DexHandCan_shared/`, `tools/make_sketches.py` | 두 스케치의 공통 원본과 재생성 도구 |
| `firmware/tests/ack_test.cpp` | 펌웨어 로직 네이티브 테스트 |

`protocol`은 손별 설정입니다. v2를 올리지 않은 손은 `protocol: 1`로 두세요(결과는 `SENT`). v1 손에 2로 두면 enable이 `NO_REPLY`로 거부됩니다.

## 테스트

```bash
python3 -m pytest -q ros2_ws/src/bhl_hand_sdk/test
```

`test/test_firmware.py`(5개)는 각 보드의 스케치가 `dexhand.yaml`의 같은 손과 노드, 펄스 범위, 프로필 태그까지 일치하는지, 스케치가 공통 원본에서 생성된 그대로인지 확인하고 펌웨어 로직 네이티브 테스트를 돌립니다. `test/fake_hand.py`는 python-can 가상 버스에 올린 v2 손입니다(펌웨어 `HandController.h`와 같은 규칙). 22개 테스트가 ID 규칙, 프레임 바이트, 변환, 길항 쌍, dry run, v2 전달 경로(적용, 무응답 손 거부, 프로필 불일치, RESUME, LOST, 그룹 손실, 응답 손실 구분, 늦은 응답, HELD_ELSEWHERE, 범위 거부 채널, HOLD/RESUME, E-STOP, 하트비트), v1 경로, diagnostics를 확인합니다. 실제 CAN, 실제 손, ROS 노드 실행 확인은 아직 하지 않았습니다.
