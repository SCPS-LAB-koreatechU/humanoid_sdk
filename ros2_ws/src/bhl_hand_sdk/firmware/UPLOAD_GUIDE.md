# DexHand 펌웨어 업로드 가이드 (프로토콜 v2)

업로드 전에 [SIM_GUIDE.md](../SIM_GUIDE.md)의 가상 손 보드로 SDK 동작을 먼저 확인할 수 있습니다. 가상 보드는 이 스케치들과 같은 펌웨어 헤더로 만듭니다.

손마다 ESP32 보드가 하나씩 있고, **보드마다 올리는 스케치가 다릅니다.** 두 스케치는 손가락 보정값(펄스 범위)과 CAN 노드 번호가 달라서, 바꿔 올리면 손가락이 범위 밖으로 당겨지거나 SDK가 다른 손으로 인식합니다.

## 1. 어느 보드에 무엇을 올리나

| 보드 | 올릴 폴더 | CAN 노드 | PC → 손 ID (제어, 목표, PING) | 손 → PC ID (STATUS, PONG) | 프로필 태그 | `dexhand.yaml` |
|---|---|---:|---|---|---|---|
| 왼손 ESP32 | `firmware/DexHandLeft/` | 60 | 0x03C, 0x33C, 0x23C | 0x2BC, 0x1BC | `c3c778d2` | `hands.left` |
| 오른손 ESP32 | `firmware/DexHandRight/` | 61 | 0x03D, 0x33D, 0x23D | 0x2BD, 0x1BD | `c7b282e4` | `hands.right` |

- 폴더 하나가 스케치 하나입니다. 폴더 안의 `.ino`와 헤더 5개(`BoardConfig.h`, `HandController.h`, `PoseReceiver.h`, `ServoOutput.h`, `StatusReport.h`)를 함께 올립니다. 각 손 폴더의 `.ino`를 Arduino에서 열면 됩니다.
- 왼손 폴더에 오른손 `BoardConfig.h`가 섞이면 컴파일 단계에서 막힙니다(`static_assert(kNodeId == 60, ...)`).
- 프로필 태그는 그 손의 `BoardConfig.h`에 있는 `kProfileSha256` 앞 8자리입니다. 업로드 후 PING 응답에 이 값이 실려 오므로, 어느 보드에 어느 펌웨어가 올라갔는지 PC에서 확인할 수 있습니다(5절).
- `DexHandCan_shared/`는 두 스케치의 공통 원본이고 직접 올리지 않습니다. `tests/`, `tools/`도 PC용입니다.
- 이 펌웨어에 포함되지 않은 것: 손목 SCS2332 서보.

**보드에 라벨을 붙여 두세요.** 두 보드는 같은 ESP32-WROOM-32E이고 GPIO 배치도 같아서 겉으로는 구분되지 않습니다. 처음 한 번은 아래 5절의 PING으로 확인하고, 그 다음 "L / 노드 60", "R / 노드 61"처럼 보드에 적어 두는 것을 권합니다.

## 2. 준비

| 항목 | 값 |
|---|---|
| 보드 | ESP32-WROOM-32E 계열 (classic ESP32. S3, C3는 안 됨) |
| Arduino 코어 | esp32 by Espressif Systems 3.x. 연구실 기준 3.3.12, 3.3.2에서도 컴파일 확인 |
| 보드 선택 | "ESP32 Dev Module", FQBN `esp32:esp32:esp32` |
| 외부 라이브러리 | 없음 (CAN은 내장 TWAI, PWM은 내장 LEDC) |
| 케이블 | 데이터 전송이 되는 USB 케이블 (충전 전용 케이블은 포트가 안 잡힘) |

arduino-cli를 쓰는 경우 다음과 같이 준비합니다.

```bash
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32@3.3.12
```

Arduino IDE를 쓰는 경우: 환경설정의 추가 보드 관리자 URL에 위 주소를 넣고, 보드 매니저에서 "esp32"를 설치한 뒤 보드를 "ESP32 Dev Module"로 고릅니다.

## 3. 업로드 전 안전 조치

1. **서보 전원을 끕니다.** 업로드 중에 ESP32가 리셋되면서 PWM 핀이 잠깐 뜰 수 있어서, 서보가 튈 수 있습니다.
2. 팔 제어(`arm_controller`, `hand_controller`)를 멈춥니다. 업로드 중 손 보드가 CAN에서 잠시 사라지므로, 실행 중인 노드가 `LOST`나 `NO_REPLY`를 냅니다.
3. CAN 배선은 연결해 둬도 됩니다. 업로드는 USB로만 합니다.

## 4. 업로드

`humanoid_sdk/ros2_ws/src/bhl_hand_sdk/firmware`에서 실행합니다. 포트는 예시이므로 실제 포트로 바꿉니다. Linux는 `/dev/ttyUSB0` 또는 `/dev/ttyACM0`, Windows는 `COM5` 같은 형태입니다.

```bash
# 왼손 보드만 USB로 연결한 상태에서
arduino-cli compile --fqbn esp32:esp32:esp32 DexHandLeft
arduino-cli upload  --fqbn esp32:esp32:esp32 --port /dev/ttyUSB0 DexHandLeft

# 오른손 보드로 바꿔 연결한 다음
arduino-cli compile --fqbn esp32:esp32:esp32 DexHandRight
arduino-cli upload  --fqbn esp32:esp32:esp32 --port /dev/ttyUSB0 DexHandRight
```

**보드는 한 번에 하나만 USB로 연결하세요.** 두 개를 동시에 꽂으면 포트를 헷갈리기 쉽습니다.

컴파일이 정상이면 다음처럼 나옵니다(3.3.2 기준, 버전에 따라 조금 다름).

```
Sketch uses 285315 bytes (21%) of program storage space. Maximum is 1310720 bytes.
Global variables use 21152 bytes (6%) of dynamic memory, ...
```

업로드가 안 될 때 확인할 것:

| 증상 | 확인 |
|---|---|
| 포트가 안 보임 | 데이터 케이블인지, USB-UART 드라이버(CP210x 또는 CH340)가 있는지, Linux면 `dialout` 그룹인지(`sudo usermod -aG dialout $USER` 후 재로그인) |
| `Failed to connect to ESP32: Wrong boot mode` | BOOT 버튼을 누른 채 업로드를 시작하고, "Connecting..."이 나오면 뗌 |
| `Use a classic ESP32-WROOM board and Arduino-ESP32 3.x` 오류 | 보드 선택이 ESP32 Dev Module인지, 코어가 3.x인지 |
| `Wrong CAN node for DexHand...` 오류 | 폴더 안 `BoardConfig.h`가 다른 손 것임. 이 저장소의 파일로 되돌림 |

## 5. 업로드 후 확인

1. USB를 뽑고 배터리 전원을 넣습니다(서보 전원은 아직 끈 상태). 펌웨어는 자동으로 실행되고, 첫 목표가 오기 전까지는 PWM을 내보내지 않습니다.
2. 로봇 PC에서 CAN을 올립니다: `ros2_ws/src/bhl_bringup/scripts/can_up.sh can0`
3. `config/dexhand.yaml`에서 해당 손의 `protocol`이 2인지 확인합니다.
4. PING으로 보드와 펌웨어를 확인합니다.

```bash
ros2 run bhl_hand_sdk hand_cli left ping
#   PONG: protocol 2, profile c3c778d2 (matches config), flags 0x00
ros2 run bhl_hand_sdk hand_cli right ping
#   PONG: protocol 2, profile c7b282e4 (matches config), flags 0x00
ros2 run bhl_hand_sdk hand_cli right listen --seconds 2     # 0.5 s마다 HEARTBEAT
```

| 결과 | 뜻과 조치 |
|---|---|
| `matches config` | 맞는 보드에 맞는 펌웨어가 올라감 |
| `left ping`에 `profile c7b282e4 (config says c3c778d2!)` | 왼손 보드에 오른손 펌웨어가 올라감. 다시 올림 |
| `left ping`은 무응답, `right ping`이 둘 다 응답 | 두 보드 모두 오른손 펌웨어. 왼손 보드에 `DexHandLeft` 다시 올림 |
| 둘 다 `no PONG` | 배터리 전원, CAN-H/L 배선과 GND, 트랜시버 TXD/RXD(GPIO5/GPIO34), `can0` 상태 확인. v1 펌웨어나 다른 펌웨어가 그대로 있어도 응답하지 않음 |
| `flags 0x01` | PWM 출력 중(이미 목표를 받은 상태) |

5. 서보 전원을 켜고 작은 동작부터 확인합니다.

```bash
ros2 run bhl_hand_sdk hand_cli right grip pinch --level 0.2 --seconds 2
#   enable: APPLIED ...
#   N sets: APPLIED N; latency median x.xx ms, max x.xx ms; acks lost 0
```

`NO_REPLY`가 섞여 나오면 지연 값과 `late` 수를 보고 `reply_timeout`을 조정합니다(`config/dexhand.yaml`). `REJECT_RANGE`가 나오면 보드의 `BoardConfig.h`와 `dexhand.yaml`의 `open_us`, `close_us`가 서로 다른 것입니다.

## 6. 한 손만 v2로 바꿀 때

`protocol`은 손별 설정이라, 한 손은 v2, 다른 손은 v1로 운용할 수 있습니다.

```yaml
hands:
  left:  {protocol: 1, ...}   # 아직 v1 펌웨어: 전달 결과는 SENT(확인 불가)
  right: {protocol: 2, ...}   # v2 업로드 완료
```

v1 펌웨어로 되돌릴 때는 dexhand_control의 `firmware/DexHandLeft`, `firmware/DexHandRight`를 같은 방법으로 올리고 `protocol: 1`로 바꿉니다. PC → 손 프레임은 두 버전이 같으므로, dexhand_control의 `HandSender`도 v2 펌웨어와 그대로 동작합니다.

## 7. 보정값이 바뀌었을 때

손가락 보정값의 원본은 dexhand_control의 `profiles/left_hand.json`, `profiles/right_hand.json`입니다. 값을 바꾸면 다음 순서로 맞춥니다.

1. dexhand_control에서 `python tools/generate_board_config.py`로 해당 손의 `BoardConfig.h`를 다시 만듭니다.
2. 새 `BoardConfig.h`를 이 저장소의 `firmware/DexHandLeft/` 또는 `firmware/DexHandRight/`에 복사합니다.
3. `python3 tools/make_sketches.py`를 실행합니다(공통 소스가 바뀌었을 때만 실제로 파일이 바뀜).
4. `config/dexhand.yaml`의 `open_us`, `close_us`, `profile_tag`(새 `kProfileSha256` 앞 8자리)를 맞춥니다.
5. `python3 -m pytest -q test/test_firmware.py`를 실행합니다. 노드, 범위, 태그 중 하나라도 어긋나면 실패합니다.
6. 해당 보드에 다시 올리고 5절의 PING으로 새 태그를 확인합니다.

dexhand_control 쪽에서 직접 v2를 만들려면 `DexHandCan_shared/`의 헤더 4개와 `DexHand.ino.in`을 그쪽 `firmware/DexHandCan/`에 반영하고, `tools/prepare_firmware.py`가 `StatusReport.h`도 스케치 폴더로 복사하도록 고쳐야 합니다.

## 8. 아직 확인하지 않은 것

- 실제 보드 업로드, 배터리 부팅, 실제 CAN 응답과 지연 (이 저장소 작업에서는 컴파일과 PC 테스트까지만 함)
- 왼손 보드의 실제 CAN 핀. 기본값 GPIO5/GPIO34는 도면 기준이고 납땜 상태는 확인하지 않았음
- 오른손 실물 배선
