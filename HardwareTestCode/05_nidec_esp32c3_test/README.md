# Nidec ESP32-C3 테스트

`05_nidec_esp32c3_test.ino`를 Arduino IDE에서 여세요.

기준: `04_nidec_instructables_port.ino`, `Nidec_ESP32C3_테스트.md`.
기존 하드웨어 테스트 폴더인 `HardwareTestCode/`에 작성했습니다.
Uno Timer1 코드를 ESP32 LEDC로 변경했으며, 배선에 없는 가변저항과 Hall ADC 입력은 사용하지 않습니다.

| 신호 | GPIO | 동작 |
|---|---|---|
| PFM / Yellow | 4 | 80% 듀티, 기본 1000 Hz |
| Enable / Blue | 5 | HIGH 구동, 부팅 시 LOW |
| Direction / Green | 6 | 기본 HIGH, `dir 0` 또는 `dir 1` |
| Brake / White | 7 | 기본 입력(내부 풀업 없음), 별도 시험 |
| FG / Orange | 10 | 개조된 FG, INPUT_PULLUP + FALLING |

## 업로드 및 첫 구동 (약 5분)

1. Arduino-ESP32 2.x 또는 3.x에서 실제 ESP32-C3 보드(예: ESP32C3 Dev Module)와 포트를 선택합니다. 네이티브 USB 사용 시 USB CDC On Boot를 Enabled로 설정합니다.
2. 배선 문서의 전원·로직 레벨 사전 확인을 마칩니다. 12V 전원과 USB 전원은 분리하고 GND를 공통으로 연결합니다. FG의 전압도 GPIO 연결 전에 3.3V 입력에 맞는지 확인합니다.
3. 업로드 후 시리얼 모니터를 115200 baud, 새 줄 전송으로 엽니다. 부팅 직후 Enable과 PFM은 LOW이며 자동 구동하지 않습니다.
4. `f 1000`을 전송하고 `run`을 전송합니다. `f 2000`, `f 3000`처럼 수동으로 높이며 FG를 관찰합니다. 허용 입력 범위 250~26000 Hz는 탐색 범위이며 실제 모터의 검증된 동작 범위가 아닙니다.
5. `stop` 또는 `!`로 정지합니다. `!`는 새 줄 없이도 처리되지만 이후 명령 전에는 새 줄을 전송해야 합니다. Enable LOW는 회전자의 즉시 정지를 보장하지 않습니다.

## 방향 및 브레이크 (구동 확인 후 별도 시험)

1. `stop` 후 실제 회전이 완전히 멎을 때까지 기다립니다. `dir 0` 또는 `dir 1`을 전송한 뒤 `run`으로 해당 방향을 확인합니다. 방향 명령은 자동으로 정지시키며 재시작하지 않습니다.
2. 브레이크 사전 전압·극성 확인 후 `brake high` 또는 `brake low`를 전송합니다. 명령은 먼저 Enable/PFM을 끄므로 이것만으로는 브레이크 효과를 분리해 판정할 수 없습니다. 설정한 레벨에서 명시적으로 `run`을 전송하여 구동 억제 여부를 비교합니다. LOW 제동은 원본 주석의 가정이며 실측 확정이 아닙니다.
3. `brake off`로 GPIO7을 입력 상태로 복귀합니다. 이는 강제로 브레이크를 해제하는 명령이 아니며, 실제 상태는 모터 입력 회로에 달려 있습니다. 브레이크 설정은 `stop` 후에도 유지됩니다.

## 측정값

매초 `set_hz`, LEDC 실제 설정값 `pfm_hz`, `fg_count`, `fg_hz`, `rpm_fg`, `rpm_command_est`를 출력합니다. FG는 원자적으로 카운터를 복사·초기화하고 실제 측정 간격으로 주파수를 계산합니다. FG가 없으면 0 Hz를 표시하며 자동 정지하지 않습니다.

`rpm_command_est = PFM / 6.6667`은 원본의 **지령 기반 추정치**입니다. 실제 FG RPM과 같다고 가정하지 않습니다. 회전당 FALLING 펄스 수가 확인되지 않았으므로 기본 `rpm_fg=NA`입니다. 실제 한 바퀴의 펄스 수 또는 외부 회전계로 보정한 값을 `ppr N`으로 설정하면 `rpm_fg = fg_hz × 60 / N`을 표시합니다. `ppr 0`은 미확정 상태로 돌아갑니다.

설정은 RAM에만 보관되어 재부팅하면 초기값으로 복귀합니다. 하드웨어 자동 탐색·자동 방향 반전은 수행하지 않습니다.

## PWM 초기화 오류 진단

Arduino-ESP32 3.x에서는 듀티 반영 직후의 `ledcReadFreq()`가 아직 0을 반환할 수 있어 Enable LOW 상태로 최대 20 ms 기다린 뒤 판정합니다. GPIO는 출력 모드 설정 후 값을 씁니다. `run` 실패 시 일반 오류 바로 앞의 `ledcAttach`, `ledcWrite`, `frequency readback` 상세 오류로 실패 단계를 구분할 수 있습니다. 이 대기 수정의 실제 보드 동작은 재업로드 후 확인해야 합니다.
