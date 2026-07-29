# Self_Balancing_Device

## ESP32-S3 보드 핀아웃

아래 핀아웃은 ESP32-S3 개발 보드 기준의 배치 참고용입니다. BMI270 6개(SPI 공통 3핀 + CS 6핀), 서보모터 3개(PWM 신호선), CAN 통신(TX/RX 2핀)의 핀 배정을 반영했습니다.
```text
                                                     +--[ANT]--+
                                      3V3            |         | GND
                                      3V3            |         | GPIO43        (USB-UART TXD0, 사용 주의)
                                      RST            |         | GPIO44        (USB-UART RXD0, 사용 주의)
           BMI270 x6 SPI SPC / SCK    GPIO4          |         | GPIO1         (CAN TX)
           BMI270 x6 SPI SDI / MOSI   GPIO5          |         | GPIO2         (CAN RX)
           BMI270 x6 SPI SDO / MISO   GPIO6          |         | GPIO42
           BMI270_1 CS                GPIO7          |         | GPIO41
           BMI270_2 CS                GPIO15         |         | GPIO40
           BMI270_3 CS                GPIO16         |         | GPIO39        (JTAG 예비)
           BMI270_4 CS                GPIO17         |         | GPIO38        (내장 PSRAM 사용 주의)
           BMI270_5 CS                GPIO18         |         | GPIO37        (내장 PSRAM 사용 주의)
           BMI270_6 CS                GPIO8          |         | GPIO36        (내장 PSRAM 사용 주의)
                                      GPIO3          |         | GPIO35        (내장 PSRAM 사용 주의)
                                      GPIO46         |         | GPIO0         (BOOT, 사용 주의)
           BMI270_1,2 INT             GPIO9          |         | GPIO45        (VDD_SPI, 사용 주의)
           BMI270_3,4 INT             GPIO10         |         | GPIO48        ( 보드 RGB LED 미사용)
           BMI270_5,6 INT             GPIO11         |         | GPIO47       
            서보모터1 PWM             GPIO12         |         | GPIO21       
            서보모터2 PWM             GPIO13         |         | GPIO20        (USB D-, 사용 금지)
            서보모터3 PWM             GPIO14         |         | GPIO19        (USB D+, 사용 금지)
                                      5V             |         | GND
                                      GND            |         | GND
                                                     +-[UART][USB]-+
```

### 핀 배정 현황

- BMI270 6개: SPI 공통 3핀(GPIO4/5/6) + CS 6핀(GPIO7/15/16/17/18/8) 배정 완료. 물리적 배선 필요.
- BMI270 INT (축별 공유): 1축(BMI270_1,2) → GPIO9, 2축(BMI270_3,4) → GPIO10, 3축(BMI270_5,6) → GPIO11. 각 축 내 2개 센서의 INT1을 open-drain로 설정해 같은 GPIO에 wired-OR로 묶음(내부 풀업 또는 외부 풀업 저항 필요).
- CAN 통신: GPIO1(TX), GPIO2(RX) 배정 완료. CAN 트랜시버(예: SN65HVD230) 연결 필요.
- 서보모터 3개: GPIO48(서보3), GPIO47(서보1), GPIO21(서보2) 배정 완료(3핀 연속 배치). PWM 신호선 배선 필요. 서보 전원(5V/GND)은 별도 외부 공급 필요.

### 핀 배정 재점검 (충돌 확인)

새로 배정한 CAN·서보·INT 핀과 기존 BMI270/보드 예약 핀 사이에 겹치는 부분이 없는지 다시 확인한 결과입니다.

- BMI270 SPI/CS 사용 핀(GPIO4,5,6,7,8,15,16,17,18)과 CAN/서보/INT 신규 핀(GPIO1,2,48,47,21,9,10,11) 사이에 중복 없음.
- GPIO1, GPIO2, GPIO47, GPIO21은 기존에 "예비"로 표시되어 있던 핀으로 다른 용도와 충돌 없음.
- GPIO9, GPIO10, GPIO11은 별도 기능(JTAG/PSRAM/스트래핑)이 걸려 있지 않은 순수 예비 핀이라 INT 용도로 충돌 없음.
- 서보 3핀을 연속 배치하기 위해 서보3을 GPIO40(JTAG 겸용) 대신 GPIO48로 변경. GPIO48은 보드 RGB LED 겸용 핀이라 서보3으로 쓰면 온보드 RGB LED 기능은 포기해야 함(별도 외장 LED로 대체 가능). GPIO40은 다시 미사용 예비 핀으로 남음.
- GPIO0(BOOT), GPIO45(VDD_SPI), GPIO19/20(USB D+/D-), GPIO43/44(USB-UART), GPIO35~38(내장 PSRAM)에는 신규 핀을 배정하지 않아 부팅/USB/PSRAM 관련 충돌 없음.
- GPIO3, GPIO46은 ESP32-S3 스트래핑 핀(GPIO46은 입력 전용)이라 이번에도 배정하지 않고 미사용 상태로 남겨둠. 출력이 필요한 용도(PWM 등)로는 사용하지 말 것.
- ESP32-S3는 대부분의 GPIO가 LEDC(PWM) 매트릭스 라우팅을 지원하므로 GPIO48/47/21을 서보 PWM으로 사용하는 데 기능적 제약 없음.
- 같은 축 2개 센서의 INT1을 한 GPIO에 묶을 때는 두 센서 모두 open-drain 설정이 필요하며, 한쪽 센서가 인터럽트를 클리어하지 못하면 같은 축의 다른 센서 인터럽트도 가려질 수 있음(축별로 묶여 있어 다른 축에는 영향 없음).
- 결론: 현재 배정된 핀 구성(BMI270 9 + CAN 2 + 서보 3 + INT 3 = 17핀)에서는 충돌 없음. 서보 3핀은 GPIO48/47/21로 연속 배치됨.

### BMI270 SPI 핀 설명

| BMI270 핀 | ESP32-S3 연결 | SPI 모드 역할 |
| --- | --- | --- |
| VCC | 3V3 | 센서 전원 입력 |
| GND | GND | 전원 접지 |
| SCL / SPC | GPIO4 공통 | SPI 클럭 SCK |
| SDA / SDI | GPIO5 공통 | SPI MOSI. ESP32-S3에서 BMI270으로 데이터 입력 |
| SA0 / SDO | GPIO6 공통 | SPI MISO. BMI270에서 ESP32-S3로 데이터 출력 |
| CS | 센서별 개별 연결 | 같은 SPI 버스에서 BMI270 6개를 구분하는 칩 선택 핀 |
| INT1 | 미연결 | 센서 인터럽트 출력 1. 필요 시 추후 배정 |
| INT2 | 미연결 | 센서 인터럽트 출력 2. 필요 시 추후 배정 |


BMI270 6개는 SPI 클럭, MOSI, MISO를 공유하고 CS 핀만 센서별로 따로 연결합니다. INT1/INT2는 현재 연결하지 않고, 데이터 준비 인터럽트가 필요해질 때 별도 GPIO를 배정합니다.

## 프로젝트 개요


### 모터 시스템
- **BLDC 모터** 사용



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





https://oshwhub.com/iMcHineSe/mini_simplefoc
https://oshwhub.com/flowersauce/simplefoc4008
https://oshwhub.com/flowersauce/drv8313

https://oshwhub.com/yourallo/youngfoc


드라이버는 여러가지 고민 중 심플하게 simpleFOC 드라이버로 진행. 
모터는 RC용 BLDC 모터를 쓸 수 도 있지만
기존에 구입했던 저렴이  BLDC 모터에 simpleFOC 드라이버를 연결.

이경우 simplefoc 드라이버에 신호를 주려면 in1, in2,in3  3pwm 필요하며, 
en, nfault, nsleep, nreset 핀 연결도 필요

제대로 FOC 모드를 사용하려면 ina240 전류센서 3개를 UVW 각상에 연결해야 한다.
이경우 adc 핀 3개도 필요.

이 경우 foc 드라이버에서 fault, sleep, reset 은 무시하고, 
in1,in2,in3 3pwm 핀과 ina240 3핀(adc), en만 연결하는 것으로 진행할 예정.
축당 3pwm과 adc 3개, en 1개 핀 필요.

모터가 총 3개가 될 예정이므로 디지털 io 핀이 9개, adc 핀이 9개, en 핀이 1개(3모터를 하나로 묶음) 필요.
자기각 센서를 
추가로 자기각 센서 as5047p 는 spi 통신이므로 cs, clk, miso, mosi 4핀 필요하며 
3개 구성시 cs 핀만 추가하면 된다. 
총 4핀(spi) + cs2 + cs3 = 6핀 필요

총 필요한 핀 수
- 디지털 io: 9핀
- adc: 9핀
- en: 1핀
- spi: 6핀
총 25핀 필요.







코드 참조
https://oshwhub.com/45coll/lai-luo-san-jiao-xing-3205-ban-ben-you-hua-bu-fen-dian-lu

## 참고 자료

https://oshwhub.com/45coll/zi-ping-heng-di-lai-luo-san-jiao_10-10-ban-ben
https://gitee.com/coll45/foc -> https://github.com/LeeMakeKR/FOC_fromgitee 로 클론. 



