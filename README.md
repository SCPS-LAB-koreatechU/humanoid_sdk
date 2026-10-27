# humanoid_sdk

Berkeley Humanoid Lite 상반신(양팔 recoil 모터, CAN)을 ROS 2 Humble에서 띄우고 다루는 워크스페이스입니다. 로봇에 실린 컴퓨터가 현재 상태를 내보내고, 나머지는 전부 그 상태를 보고 통신합니다.

| 패키지 | 역할 |
|---|---|
| [`bhl_bringup`](ros2_ws/src/bhl_bringup) | bring-up. `robot.launch.py`(로봇 컴퓨터: `arm_controller` + `joint_state_publisher` + `robot_state_publisher` → `/joint_states`, `/tf`, `/robot_description`, `/diagnostics`), `remote.launch.py`(원격 PC: RViz, 슬라이더), `demo.launch.py`(한 PC). 네트워크, CAN, 자동 시작 설정 |
| [`bhl_arm_sdk`](ros2_ws/src/bhl_arm_sdk) | 모터별 각도 제어 노드 `arm_controller`(CAN ↔ 관절 상태, 목표 토픽, 파라미터, 서비스, 안전 장치), 슬라이더 GUI, G1 예제식 시퀀스, 관절 맵 `config/bhl_arms.yaml` |

```
humanoid_sdk/
├── env.sh            source 하면 ROS 2, description 워크스페이스, 이 워크스페이스, ROS_DOMAIN_ID
├── humanoid.repos    URDF + 메시(berkeley_humanoid_lite_description)를 vcstool로 가져오는 목록
└── ros2_ws/src/
    ├── bhl_bringup/
    └── bhl_arm_sdk/
```

## 빠른 시작

```bash
git clone https://github.com/SCPS-LAB-koreatechU/humanoid_sdk ~/humanoid_sdk
cd ~/humanoid_sdk/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select bhl_arm_sdk bhl_bringup
source ../env.sh

ros2 launch bhl_bringup demo.launch.py                    # 연습 모드 + RViz + 슬라이더 (CAN 안 엶)
```

RViz에 로봇을 그리고 `/tf`를 내려면 URDF가 필요합니다: `Berkeley-Humanoid-Lite-origin-`의 `ros2_ws`를 따로 빌드해 두거나(`env.sh`가 `~/Berkeley-Humanoid-Lite/ros2_ws`를 찾음), `vcs import src < ../humanoid.repos`로 이 워크스페이스에 가져옵니다. 없으면 팔 제어만 되고 RViz는 빠집니다.

로봇 컴퓨터와 원격 PC로 나누는 법, 토픽 계약, CAN 자동 설정, 실물 bring-up 순서는 [`bhl_bringup/README.md`](ros2_ws/src/bhl_bringup/README.md), 모터 제어 자체(각도 지정, 안전 장치, 설정값)는 [`bhl_arm_sdk/README.md`](ros2_ws/src/bhl_arm_sdk/README.md).

## 테스트 (ROS, CAN 없이)

```bash
python3 -m pytest -q ros2_ws/src/bhl_arm_sdk/test ros2_ws/src/bhl_bringup/test
```

## 관련 저장소

- [Berkeley-Humanoid-Lite-origin-](https://github.com/SCPS-LAB-koreatechU/Berkeley-Humanoid-Lite-origin-) — URDF, 메시, MoveIt 설정, PC 쪽 `hardware_bridge.py`
- [Berkeley-humanoid-lite `feature/onboard-x5`](https://github.com/SCPS-LAB-koreatechU/Berkeley-humanoid-lite/tree/feature/onboard-x5) — RDK X5용 `bhl_onboard`(unitree_sdk2식 `/lowcmd`, `/lowstate`). 그쪽 `/bhl/joint_states`도 `bhl_bringup`의 `joint_state_sources`로 받을 수 있습니다
