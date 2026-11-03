# 손 시뮬레이션 가이드

보드에 펌웨어를 올리기 전에 PC에서 손 제어를 확인하는 방법입니다. 두 부분으로 나뉩니다.

| 부분 | 하는 일 | 실체 |
|---|---|---|
| 가상 손 보드 `hand_sim` | 손 ESP32 대신 PC가 CAN 프레임에 응답 | 실제 펌웨어 헤더(`firmware/DexHandCan_shared/`)와 각 손의 `BoardConfig.h`를 PC용으로 컴파일한 것. PWM은 로그로, TWAI는 SocketCAN 또는 파이프로 바꿈 |
| 관절 추정 시각화 `hand_joint_estimate` | 명령한 펄스를 손가락 관절각으로 바꿔 RViz에 그림 | `config/dexhand.yaml`의 `visualization` 선형 매핑 |

## 무엇을 확인할 수 있고, 무엇은 안 되나

| 확인 가능 | 확인 불가 |
|---|---|
| SDK와 펌웨어 사이 프로토콜 전체 (APPLIED, REJECT_*, NO_REPLY, CONTROL_ACK, PONG, HEARTBEAT) | 서보가 실제로 도는지, 손가락이 실제로 그 각도까지 가는지 |
| 보드 꺼짐, 프레임 손실, 응답 손실, 출력 실패, 잘못된 스케치 업로드 같은 고장 상황 | 힘줄 장력, 길항 쌍 간섭, 실제 파지 성공 |
| ROS 노드, 서비스, 토픽, launch 연동, E-STOP 연동 | 실제 CAN 버스 타이밍, USB-CAN 어댑터 지연 |
| grip 값이 대략 어떤 모양이 되는지 (RViz) | 그 모양이 실제 손과 같은지 (매핑은 가설) |

RViz에 보이는 손은 **명령의 그림**이지 측정값이 아닙니다.

## 준비

- `colcon build --symlink-install --packages-select bhl_hand_sdk bhl_bringup`
- C++17 컴파일러(g++ 또는 clang++). `hand_sim`은 처음 쓸 때 `~/.cache/bhl_hand_sdk/`에 자동으로 빌드되고, 펌웨어 소스가 바뀌면 다시 빌드됩니다.
- 손가락 전체 화면(`hand_view`)에는 Berkeley-Humanoid-Lite description 워크스페이스의 `vendor/dexhandv2_description`이 필요합니다. `BHL_DESCRIPTION_WS`(기본 `~/Berkeley-Humanoid-Lite/ros2_ws`)에서 찾습니다. 메쉬는 복사하지 않습니다(CC BY-NC-SA 4.0).

## 방법 1. CAN 없이 (`channel: sim`), 가장 간단함

손 노드가 자기 손의 가상 보드를 직접 띄우고 파이프로 대화합니다. 커널 CAN, `sudo`, `vcan`이 필요 없습니다.

```bash
# 손만: 양손 노드 + 가상 보드 + 관절 추정 + V2 손 RViz
ros2 launch bhl_hand_sdk sim.launch.py

# 로봇 전체 RViz: 팔(dry run) + 손(가상 보드) + 손가락 그림
ros2 launch bhl_bringup demo.launch.py hands:=both hand_channel:=sim
```

`hand_channel:=sim`이면 팔은 그대로 dry run이고, 손만 가상 보드로 실제 스트림을 보냅니다. 로봇 전체 화면에서는 로봇 URDF가 손가락의 Pitch와 Yaw만 움직이므로 엄지와 손끝은 고정으로 보입니다. 전체 관절은 `sim.launch.py`의 V2 손 화면에서 봅니다.

다른 터미널에서 조작합니다.

```bash
ros2 service call /hand_right/enable std_srvs/srv/SetBool "{data: true}"
ros2 param set /hand_right grip point
ros2 param set /hand_right grip_level 1.0
ros2 topic echo /hand_right/delivery          # {"result": "APPLIED", "sequence": ..., "latency_ms": ...}
ros2 topic echo /hand_right/status --once
```

고장 주입은 `~/sim_command` 토픽으로 합니다.

```bash
ros2 topic pub --once /hand_right/sim_command std_msgs/msg/String "{data: 'power right off'}"   # 0.2 s 뒤 LOST
ros2 topic pub --once /hand_right/sim_command std_msgs/msg/String "{data: 'power right on'}"    # 재부팅, 다시 enable
ros2 topic pub --once /hand_right/sim_command std_msgs/msg/String "{data: 'drop right 3'}"      # 응답 3개 손실 -> acks_lost
ros2 topic pub --once /hand_right/sim_command std_msgs/msg/String "{data: 'lose right 2'}"      # 그룹 2 손실 -> NO_REPLY
ros2 topic pub --once /hand_right/sim_command std_msgs/msg/String "{data: 'fail right'}"        # OUTPUT_FAIL 래치
ros2 topic pub --once /hand_right/sim_command std_msgs/msg/String "{data: 'flash right left'}"  # 왼손 스케치를 올린 상황
```

ROS 없이 벤치 도구로도 됩니다.

```bash
ros2 run bhl_hand_sdk hand_cli right ping --channel sim
ros2 run bhl_hand_sdk hand_cli right grip pinch --level 0.5 --channel sim
ros2 run bhl_hand_sdk hand_cli right grip power --channel sim --sim-command "power right off"
```

## 방법 2. 가상 CAN (`vcan0`), 실제 SocketCAN 경로까지

손 노드가 실제와 똑같이 SocketCAN 소켓을 열고, 가상 보드 하나가 양손을 맡습니다. 커널 필터와 소켓 경로까지 확인할 수 있고, 보드 터미널에서 직접 고장 명령을 칩니다. Linux 전용입니다.

```bash
# 1. 가상 CAN 만들기 (재부팅하면 다시)
sudo modprobe vcan
sudo ip link add vcan0 type vcan
sudo ip link set up vcan0

# 2. 가상 보드 (이 터미널에서 명령 입력)
ros2 run bhl_hand_sdk hand_sim --channel vcan0
#   [left] boot: left sketch, node 60, profile c3c778d2
#   [right] boot: right sketch, node 61, profile c7b282e4

# 3. 손 노드
ros2 launch bhl_hand_sdk sim.launch.py channel:=vcan0
#    또는 로봇 전체:  ros2 launch bhl_bringup demo.launch.py hands:=both hand_channel:=vcan0 dry_run:=false
#    (vcan0은 손만 쓰므로 팔은 실제 can0을 엽니다. 팔 하드웨어가 없으면 dry_run을 그대로 두세요.
#     dry_run이 true면 손도 CAN을 열지 않습니다.)
```

`hand_sim` 터미널 명령:

| 명령 | 상황 |
|---|---|
| `power right off` / `power right on` | 배터리 분리, 재연결(재부팅: PWM 꺼짐, 카운터 0) |
| `flash right left` | 오른손 보드에 왼손 스케치를 올림 → 노드 60이 됨 |
| `drop right 3` | 다음 응답 3개가 버스에서 사라짐 |
| `lose right 2` / `lose right none` | 자세 그룹 2를 계속 못 받음 / 해제 |
| `fail right` | 다음 PWM 기록 실패 |
| `status` | 보드별 상태와 현재 펄스 |
| `quiet` / `verbose` | 펄스 로그 끄기, 켜기 |

`--hands right`로 한 보드만, `--flash right=left`로 시작부터 잘못 올린 상태도 만들 수 있습니다.

## 시나리오별 기대 결과

| 해 볼 것 | 기대 결과 |
|---|---|
| enable → grip → `delivery` 확인 | 세트마다 `APPLIED`, `status.hand_reported.flags`에 `PWM_ON` |
| `~/hold` 후 grip 변경 | HOLD는 `CONTROL_ACK`, 손은 마지막 펄스를 유지하고 새 목표는 보내지 않음. `~/resume`에서 이어감 |
| `power right off` | 10주기(0.2 s) 뒤 `LOST`, 노드는 송신 중단 |
| `power right on` 후 enable | `PONG flags 0x00`(재부팅됨) → START → `APPLIED` |
| `flash right left` 후 enable | 오른손 노드(61)는 `NO_REPLY`로 거부. 노드 60에는 보드 두 개가 같이 응답함(`candump vcan0`에 0x1BC PONG이 두 번) |
| `drop right 1` | 그 세트는 `NO_REPLY`, 다음 세트에서 적용 횟수가 2 늘어 `acks_lost` 1 |
| `lose right 3` | 세트가 완성되지 않아 응답 없음, 계속되면 `LOST` |
| `fail right` | `OUTPUT_FAIL`, 이후 목표는 `REJECT_INHIBITED`. enable(START)로 해제 |
| 팔 `~/estop` (demo) | 손도 `estop_action`(기본 hold) 실행, `diagnostics` ERROR |

## RViz 매핑 조정

`config/dexhand.yaml`의 `visualization.joints`가 매핑입니다. 각 관절은 `coef`(채널 번호 → 당김 비율 1당 rad)와 `limits`(rad)를 가집니다.

- 기본값은 가설입니다. 손가락마다 위, 아래 힘줄이 차동 쌍이라고 가정했습니다. 둘 다 당기면 Pitch, 차이를 주면 Yaw가 되고, 끝 힘줄은 Flexor(DIP는 따라감), 엄지 오른쪽, 왼쪽 당김은 Thumb_Pitch와 Thumb_Yaw의 차동 쌍입니다.
- RViz에서 좌우가 반대로 움직이면 해당 Yaw의 `coef` 부호 두 개를 바꿉니다.
- 실물에서 각도를 재면 `coef`를 측정값으로 바꿉니다. 예: 당김 1.0에서 검지 Pitch가 70°면 각 채널 `coef`를 0.611(= 1.222 / 2)로 바꿉니다.
- 업스트림 V2 URDF의 mimic 태그는 기준 관절 이름에 `R_`이 빠져 있어서 DIP가 움직이지 않습니다. `hand_view`는 불러올 때 이 이름을 고칩니다(`bhl_hand_sdk/v2_urdf.py`).

**길항 쌍 설정과의 관계**: 위 차동 가설이 맞다면 엄지 13, 14번을 함께 당기는 것은 정상적인 엄지 굽힘입니다. 그런데 `control.antagonist_pairs`의 기본값 `[[13, 14]]`는 둘의 합이 1을 넘으면 거부합니다. 실제 손에서 13, 14번이 어떤 관계인지 확인하고 둘 중 하나를 고쳐야 합니다.

## 문제 해결

| 증상 | 확인 |
|---|---|
| `hand_sim needs a C++17 compiler` | `sudo apt install g++` |
| `no CAN interface vcan0` | 방법 2의 1단계 (`modprobe vcan`이 안 되면 `sudo apt install linux-modules-extra-$(uname -r)`) |
| `hand_view skipped: no DexHand V2 URDF` | `export BHL_DESCRIPTION_WS=<Berkeley-Humanoid-Lite>/ros2_ws` 또는 `hand_urdf:=<파일>` |
| RViz 손이 안 움직임 | `ros2 topic hz /hand_right/joint_states_estimate`(20 Hz), enable 했는지, `command_us`가 나오는지 |
| 손 노드 `NO_REPLY` (vcan0) | `hand_sim`이 같은 채널에서 실행 중인지, `candump vcan0`에 0x2BD가 보이는지 |

## 검증 상태 (2026-11-03)

- 가상 보드: PC 컴파일(`-Wall -Wextra -Werror`) 통과. 파이프 모드로 SDK와 통합 테스트 11개 통과(`test/test_sim.py`: 스트림, HOLD/RESUME, 전원 차단 → LOST → 재부팅, 잘못된 스케치, 그룹 손실과 응답 손실 구분, 출력 실패 래치, 펌웨어 범위 검사, 오래된 순번 무시, 하트비트)
- SocketCAN(vcan0) 경로는 컴파일만 확인. 작업 환경 커널에 CAN이 없어 실행은 못 함
- 관절 추정: 매핑과 URDF 로더 테스트 7개. V2 URDF와 메쉬로 오프라인 렌더링해서 open, power, pinch, point 모양을 눈으로 확인
- ROS 2 노드와 launch는 가짜 rclpy와 launch 대체 모듈로 확인. 실제 ROS 2와 RViz에서는 아직 실행하지 않음
