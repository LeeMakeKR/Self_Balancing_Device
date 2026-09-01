# Self_Balancing_Device

## ESP32-S3 보드 핀아웃

아래 핀아웃은 ESP32-S3 개발 보드 기준의 배치 참고용입니다. IMU 6개(SPI 공통 3핀 + CS 6핀), 서보모터 3개(PWM 신호선), CAN 통신(TX/RX 2핀)의 핀 배정을 반영했습니다.

IMU는 **MPU-6500** 6개를 SPI 4선(SCK/MOSI/MISO/CS) + INT1로 연결합니다. 자세한 배선은 [SPI 배선](#spi-배선) 항목 참조.

```text
                        +--[ANT]--+
                 3V3    |         | GND
                 3V3    |         | GPIO43   (USB-UART TXD0, 사용 주의)
                 RST    |         | GPIO44   (USB-UART RXD0, 사용 주의)
IMU x6 SPI MISO  GPIO4  |         | GPIO1    (CAN TX -> 트랜시버 D, 배선 확인 완료)
IMU x6 SPI MOSI  GPIO5  |         | GPIO2    (CAN RX <- 트랜시버 R, 배선 확인 완료)
 IMU x6 SPI SCK  GPIO6  |         | GPIO42
       IMU_1 CS  GPIO7  |         | GPIO41
       IMU_2 CS  GPIO15 |         | GPIO40
       IMU_3 CS  GPIO16 |         | GPIO39   (JTAG 예비)
       IMU_4 CS  GPIO17 |         | GPIO38   (내장 PSRAM 사용 주의)
       IMU_5 CS  GPIO18 |         | GPIO37   (내장 PSRAM 사용 주의)
       IMU_6 CS  GPIO8  |         | GPIO36   (내장 PSRAM 사용 주의)
                 GPIO3  |         | GPIO35   (내장 PSRAM 사용 주의)
                 GPIO46 |         | GPIO0    (BOOT, 사용 주의)
    IMU_1,2 INT  GPIO9  |         | GPIO45   (VDD_SPI, 사용 주의)
    IMU_3,4 INT  GPIO10 |         | GPIO48   (보드 RGB LED, 미사용)
    IMU_5,6 INT  GPIO11 |         | GPIO47
  서보모터1 PWM  GPIO12 |         | GPIO21
  서보모터2 PWM  GPIO13 |         | GPIO20   (USB D-, 사용 금지)
  서보모터3 PWM  GPIO14 |         | GPIO19   (USB D+, 사용 금지)
                 5V     |         | GND
                 GND    |         | GND
                        +-[UART][USB]-+
```

### 핀 배정 현황

- IMU(MPU-6500) 6개: SPI 공통 3핀(MISO GPIO4 / MOSI GPIO5 / SCK GPIO6) + CS 6핀(GPIO7/15/16/17/18/8) 배정 완료. 물리적 배선 필요.
- IMU INT (축별 공유): 1축(IMU_1,2) → GPIO9, 2축(IMU_3,4) → GPIO10, 3축(IMU_5,6) → GPIO11. 각 축 내 2개 센서의 INT1을 open-drain으로 설정해 같은 GPIO에 wired-OR로 묶음(내부 풀업 또는 외부 풀업 저항 필요). MPU-6500은 `INT_PIN_CFG`(0x37)의 `INT_OPEN` 비트로 open-drain 설정 가능.
- CAN 통신: GPIO1(TX), GPIO2(RX) 배정 및 **실물 배선 확인 완료**. GPIO1 → 트랜시버 D(TXD), GPIO2 ← 트랜시버 R(RXD)로 연결됨. 트랜시버는 SN65HVD230(3.3V), RS는 GND 직결, 버스 양 끝 120Ω 종단.
- 서보모터 3개: GPIO48(서보3), GPIO47(서보1), GPIO21(서보2) 배정 완료(3핀 연속 배치). PWM 신호선 배선 필요. 서보 전원(5V/GND)은 별도 외부 공급 필요.

## IMU 센서 (MPU-6500)

IMU는 **MPU-6500** (TDK InvenSense) 단일 기종으로 통일합니다. 수급이 쉽고 저렴하며, 라이브러리 없이 레지스터 직접 접근만으로 동작합니다.

### 주요 사양

| 항목 | 값 |
| --- | --- |
| 제조사 | TDK InvenSense |
| 축 구성 | 3축 가속도 + 3축 자이로 |
| WHO_AM_I | `0x75` → **`0x70`** |
| 부팅 시 설정 파일 업로드 | 불필요 |
| SPI 최대 클럭 | **레지스터 1 MHz** / 센서 데이터 20 MHz |
| 자이로 노이즈 밀도 | 약 0.01 dps/√Hz |
| 가속도 노이즈 밀도 | 약 300 µg/√Hz |
| 자이로 범위 | ±250 ~ ±2000 dps |
| 가속도 범위 | ±2 ~ ±16 g |
| 최대 ODR | 자이로 8 kHz / 가속도 4 kHz |
| 전원 | VDD/VDDIO 1.71~3.6V |
| 레지스터 구조 | 단일 뱅크 |

노이즈 성능이 최상급은 아니므로, 6개를 모두 MPU-6500으로 채운 상태에서는 자세 추정 필터의 신뢰 가중치를 그만큼 낮춰 잡습니다. 반작용 휠 진동이 그대로 실리는 구조라 DLPF 설정(아래 초기화 항목)이 특히 중요합니다.

### SPI 배선

브레이크아웃 모듈에는 I2C용 핀 이름이 함께 인쇄되어 있습니다. SPI로 쓸 때의 실제 역할은 아래와 같습니다.

| ESP32-S3 | 모듈 실크 표기 | 역할 |
| --- | --- | --- |
| 3V3 | VCC | 전원. 3.3V (5V 인가 금지) |
| GND | GND | 접지 |
| GPIO6 | SCL / SCLK / SPC | SPI 클럭 SCK (6개 공통) |
| GPIO5 | SDA / SDI | SPI MOSI (6개 공통) |
| GPIO4 | AD0 / SDO | SPI MISO (6개 공통) |
| GPIO7/15/16/17/18/8 | CS / NCS | 칩 선택. 센서별 개별 연결 |
| GPIO9/10/11 | INT | 축별 2개씩 wired-OR |
| GND | FSYNC | 미사용. 플로팅 방지를 위해 GND 연결 |
| 미연결 | EDA / ECL | 보조 I2C 마스터 포트(외부 지자기 센서용). 미사용 |

**SDI = 입력(MOSI), SDO = 출력(MISO)** 입니다. `AD0`는 I2C 모드에서만 주소 선택 핀이고, SPI 모드에서는 MISO로 동작합니다. GND에 묶으면 데이터가 나오지 않습니다.

SPI 모드는 Mode 0(CPOL=0, CPHA=0). **레지스터 읽기/쓰기가 1 MHz 제한**이므로 클럭은 1 MHz로 고정하는 것이 안전합니다.

### 레지스터 제어

테스트 스케치 [TestCode/06_imu_test](TestCode/06_imu_test/06_imu_test.ino)는 외부 라이브러리 없이 레지스터 직접 접근으로 읽습니다. `WHO_AM_I`(0x75) 값이 `0x70`이면 통신 정상입니다.

- **`USER_CTRL`(0x6A)의 `I2C_IF_DIS` 비트를 1로 세워 I2C를 꺼야 합니다.** 이걸 빠뜨리면 SPI 통신 중 I2C 슬레이브가 함께 반응해 값이 깨집니다. SPI로 쓸 때 가장 흔한 실수입니다.
- 초기화: `PWR_MGMT_1`(0x6B) 소프트 리셋 → 클럭 소스 PLL 지정 → `USER_CTRL` I2C 차단 → `CONFIG`(0x1A)/`ACCEL_CONFIG2`(0x1D) DLPF 설정 → `GYRO_CONFIG`(0x1B), `ACCEL_CONFIG`(0x1C) 범위 설정.
- 데이터는 `0x3B`부터 14바이트 버스트로 가속도 6 + 온도 2 + 자이로 6 순서입니다.
- 온도 환산: `raw / 333.87 + 21`
- 환산 계수: 가속도 ±16 g → 2048 LSB/g, 자이로 ±2000 dps → 16.4 LSB/dps.
- SPI 읽기는 주소 최상위 비트를 1로 세우고 바로 데이터가 나옵니다. 더미 바이트가 없습니다.
- 상위 로직(퓨전·제어)에는 가속도 g, 자이로 dps로 정규화해 넘깁니다.




### 모터 시스템
알리에서 구매한 드론용 브러시리스 모터
극쌍수 10
https://aliexpress.com/item/1005008785335978.html
7~8천원 정도에 구매 가능




### 개발 접근 방식
1축에서 2축-3축으로 단계적으로 진행하며, 1축에서 기본적인 부분을 모두 완성한 후 2축, 3축으로 확장합니다.

2-3축으로 진행될 때 1축의 튜닝값은 쉽게 조정이 가능해야 하며, 하드웨어 외부에서도 별도의 키 입력이나 무선 컨트롤로 수정이 가능해야 합니다.

## 개발 계획
이 프로젝트는 단계별로 업그레이드하여 더욱 정밀한 균형 제어를 목표로 합니다:

### 1단계: 1축 균형 장치

### 2단계: 2축 균형 장치  


### 3단계: 3축 균형 장치



## 프로젝트 문서
-  **[수학적 이론](documents/수학적_이론.md)** - 자가 균형 제어 시스템의 수학적 원리








코드 참조
https://oshwhub.com/45coll/lai-luo-san-jiao-xing-3205-ban-ben-you-hua-bu-fen-dian-lu

## 참고 자료

IMU 데이터시트

- [MPU-6500 Datasheet / Register Map (TDK InvenSense)](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Datasheet2.pdf)

https://oshwhub.com/45coll/zi-ping-heng-di-lai-luo-san-jiao_10-10-ban-ben
https://gitee.com/coll45/foc -> https://github.com/LeeMakeKR/FOC_fromgitee 로 클론. 



