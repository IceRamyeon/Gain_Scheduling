# PX4 — Gazebo SITL / 실기체에서 MATLAB PositionCtrl 실행

`PID_Drone`의 위치 제어기(`PositionCtrl`)와 경로 생성(`pathGen`, `TrajectoryPlanner`)을 **수정하지 않고** 그대로 불러서, PX4 위에서 실행하는 코드다.

- **A 방식**: 위치 제어는 MATLAB이 하고, 자세·각속도 제어는 PX4가 한다.
  - MATLAB은 `SET_ATTITUDE_TARGET`(자세 쿼터니언 + 정규화 추력)을 보낸다.
- **결과 플롯**: PX4가 남기는 ULog와 MATLAB 비행 기록을 합쳐서, 시뮬레이션과 같은 `finalize_and_save_results`로 그린다.
- SITL과 실기체 모두 같은 코드를 쓴다. 연결 포트와 경로 좌표계 원점·방향만 바꾸면 된다.

> 이 코드는 MATLAB/PX4 환경에서 아직 실행해 보지 않았다. 처음에는 SITL에서만 돌려 볼 것.

## 파일

| 파일 | 역할 |
| --- | --- |
| `px4_main.m` | 설정 + 실행 진입점 (①~⑤ 진행 후 비행 기록 `.mat` 저장) |
| `run_px4_mission.m` | 단계 ①~⑤ 실행 루프 |
| `Px4Link.m` | MAVLink 연결 (상태 수신, 위치·자세 명령, 모드 전환, 파라미터 읽기) |
| `build_reference_table.m` | `TrajectoryPlanner`를 시뮬레이션과 같은 dt로 미리 호출해 기준값 표 생성 |
| `path_to_ned.m`, `ned_to_path.m` | 경로 좌표계(시뮬레이션 좌표) ↔ PX4 Local NED 변환 |
| `ulog_to_log_data.m` | ULog → 시뮬레이션과 같은 `log_data` 구조체 (④ 구간만) |
| `plot_px4_result.m` | 비행 기록 + ULog를 읽어 시뮬레이션과 같은 그래프 출력 |

## 실행 순서 (SITL)

1. PX4 SITL을 실행한다: `make px4_sitl gz_x500`
2. QGroundControl을 켠다 (모니터링용, UDP 14550).
3. MATLAB에서 `px4_main.m`을 실행한다. 단계 ①~⑤가 자동으로 진행되고, `PX4/results/px4_run_*.mat`이 저장된다.
4. 같은 비행의 ULog를 찾는다: `PX4-Autopilot/build/px4_sitl_default/rootfs/log/<날짜>/<시간>.ulg`
5. `plot_px4_result.m`을 실행하고, 위의 두 파일을 선택한다.

필요한 MATLAB 툴박스: **UAV Toolbox** (`mavlinkio`, `ulogreader`), **Curve Fitting Toolbox** (`TrajectoryPlanner`의 `fnder`).

## 실행 단계

| 단계 | 제어 주체 | 내용 | 평가 |
| --- | --- | --- | --- |
| ① 이륙·상승 | PX4 위치제어 (Offboard 위치 셋포인트) | z = 0 → 경로 시작 고도 | ✗ |
| ② 시작점 이동 | PX4 위치제어 | 프리롤 시작점으로 이동, yaw를 경로 시작 방향으로 정렬, 2초 호버 | ✗ |
| ③ 프리롤 가속 | MATLAB `PositionCtrl` | 직선 가속, 경로 시작점에서 속도 v 도달 | ✗ (제어기 전환 과도 구간) |
| ④ 경로 추종 | MATLAB `PositionCtrl` | `TrajectoryPlanner` t = 0 ~ tf | **✓ RMSE 계산 구간** |
| ⑤ 호버·착륙 | PX4 위치제어 → AUTO.LAND | 3초 호버 후 착륙 | ✗ |

- 프리롤 길이는 L = v²/(2a) + 여유이고, 기본값은 a = 2 m/s², 여유 2 m다.
- ③이 끝나는 순간 기준 위치가 정확히 경로 시작점이고 기준 속도가 v다. 그래서 ④의 기준값이 끊기지 않고 이어진다.
- 시뮬레이션의 초기 상태 "(0, 0, −6)에서 속도 v"를 재현하기 위한 구간이다.

## 시뮬레이션과 맞추기 위해 한 것

- **κ 필터**: `TrajectoryPlanner`의 곡률 LPF는 호출 1번을 0.01 s로 가정한다.
  - 그래서 비행 중에 직접 부르지 않고, 비행 전에 시뮬레이션과 같은 순서와 dt로 표를 만든 뒤 시간으로 보간한다.
- **PositionCtrl의 dt**: 적분·미분이 `obj.dt`를 쓴다.
  - 루프마다 PX4 시간 기준으로 실제 dt를 측정해서 `posCtrl.dt`에 넣는다. 파일은 수정하지 않는다.
- **시간 기준**: 경로 시간 t와 ULog 자르기 모두 PX4 부팅 후 시간(`time_boot_ms`)을 쓴다.
  - Gazebo 실시간 배율이 1이 아니어도 시간이 어긋나지 않는다.
- **좌표계**: 시뮬레이션과 PX4 모두 NED다.
  - `cfg.px4.path_origin`과 `path_rot_deg`로 경로를 실제 장소에 배치한다. SITL 기본값은 원점 그대로, 회전 0이다.

## 근사·한계

- **추력 변환**: 정규화 추력 = `MPC_THR_HOVER × U1 / (m g)`.
  - `PositionCtrl`의 U1은 m에 비례하므로 m은 상쇄된다.
  - 실제 추력은 명령에 선형이 아니라서(`THR_MDL_FAC`) 호버 근처에서만 정확한 근사다.
- **자세 제어기**: `PID_Drone/AttitudeCtrl`(명령 LPF 포함)은 쓰지 않고 PX4 자세 제어기가 대신한다.
  - 따라서 시뮬레이션과 완전히 같은 폐루프는 아니다.
- **기체 파라미터**: Gazebo 기본 기체(x500)의 질량·관성·모터는 시뮬레이션(1.25 kg, AE450)과 다르다.
- **기울기 제한**: `PositionCtrl`의 `max_angle`(30°)이 먼저 걸리고, PX4 `MPC_TILTMAX_AIR`(기본 45°)도 적용된다.
- **루프 주기**: MATLAB은 실시간 OS가 아니라서 50 Hz 루프에 몇 ms씩 흔들림이 있다.
  - 실기체에서 노트북 + 무선 텔레메트리로 자세 명령을 보내는 것은 지연이 커서 위험하다. 컴패니언 컴퓨터 사용을 권장한다.
- **ULog**:
  - 기본 로깅 프로파일은 토픽별 기록 주기가 낮을 수 있다. `SDLOG_PROFILE`을 확인하고, 필요하면 high-rate 항목을 켤 것.
  - `ulog_to_log_data`는 ULog 행 시간이 부팅 후 시간이라고 가정한다. 범위가 맞지 않으면 오류 메시지로 알려 준다.
  - Control Inputs 그래프의 U2~U4는 PX4 정규화 토크(−1 ~ 1)이고, U1은 정규화 추력을 N으로 근사 변환한 값이다.
- **조종기 없이 ARM 거부 시**: PX4 콘솔에서 `COM_RC_IN_MODE` 등 RC 관련 파라미터를 확인할 것. 값의 의미는 PX4 버전 문서로 확인.
- **안전 중단**: 위치 오차 8 m 초과 또는 기울기 60° 초과면 ③④를 중단하고 ⑤로 넘어간다.
  - ①② 시간 초과는 오류로 멈춘다. 그때 PX4는 Offboard 신호 끊김 failsafe로 동작한다.
