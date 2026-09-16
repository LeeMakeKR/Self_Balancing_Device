# Nidec 24H055M020 — ESP32-C3 연결 테스트

> 목적: [Nidec.md](Nidec.md)에서 정리한 12핀 커넥터(3.1절)를 기준으로, ESP32-C3 보드로 이 모터를 실제 구동/피드백 테스트하기 위한 배선 계획과 체크리스트.
>
> **전제조건**: 이 개체는 이미 개조되어 **8번 핀(Orange, 원래 HU)에서 BD63000 내장 FG 클린 펄스가 출력되는 상태**입니다 (Nidec.md 3.2절 개조법 참고). 따라서 아래 계획은 Hall 원신호(6/7번) 프로빙이 아니라 **FG 핀(8번) 직접 인터럽트 카운트**를 전제로 합니다.

## 1. 테스트 목표

- PFM 지령 주파수 → 실제 FG 피드백 RPM 상관관계 실측 (Nidec.md 2절/3.2절의 "1kHz 증가당 150RPM" 검증)
- Enable / Direction / Brake 제어가 ESP32-C3의 3.3V GPIO로 직접 동작하는지 확인



## 4. 신호선 ↔ ESP32-C3 GPIO 매핑

| 핀 | 색상 | 기능 | ESP32-C3 GPIO | 연결 방식 |
|---|---|---|---|---|
| 1, 2 | Red | VCC(12V) | — | 12V 전원에 직결 (ESP32 아님) |
| 3, 4 | Black | GND | GND | 공통 그라운드 |
| 5 | Violet | GND(Hall) | — | 미사용 (Hall 원신호 안 씀) |
| 6 | Grey | HW 원신호 | — | 미사용 |
| 7 | Brown | HV 원신호 | — | 미사용 |
| 8 | Orange | **FG(개조 완료)** | GPIO10 | 인터럽트 입력, `INPUT_PULLUP` + FALLING |
| 9 | Green | Direction | GPIO6 | 3.3V 직결 우선 시도 → 오동작 시 레벨시프터 |
| 10 | White | Brake | GPIO7 | 3.3V 직결 우선 시도, **미검증이므로 멀티미터 선확인 필수** |
| 11 | Blue | Enable | GPIO5 | 3.3V 직결 우선 시도 → 오동작 시 레벨시프터 |
| 12 | Yellow | PFM(속도) | GPIO4 | `ledc` 하드웨어 PWM, **80% 듀티**(50% 아님, 위 3절 참고), 주파수 가변 |

- 스트래핑 핀(GPIO2/8/9), USB 핀(GPIO18/19)은 회피
- GPIO1, GPIO3은 예비(전류 감지 등 추후 확장용)로 비워둠


