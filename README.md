# Self_Balancing_Device

## 프로젝트 개요
자이로 센서와 밸런스 휠 모터를 이용하여 모서리로 서는 자가 균형 장치를 개발하는 프로젝트입니다.

모터 컨트롤러는 모터의 회전을 정밀히 측정해야 하며, 가속도 센서는 IMU6050이 내장된 센서보드(GY-87)를 사용합니다.

### 모터 시스템
- **BLDC 모터** 사용
  - **컨트롤러 내장형**: PWM/Direction 컨트롤 방식, 모터 회전수는 별도 선으로 피드백
  - **컨트롤러 비내장형**: SimpleFOC 컨트롤러(DRV8313)와 자기각 센서(AS5600) 사용

### 개발 접근 방식
1축에서 2축-3축으로 단계적으로 진행하며, 1축에서 기본적인 부분을 모두 완성한 후 2축, 3축으로 확장합니다.

2-3축으로 진행될 때 1축의 튜닝값은 쉽게 조정이 가능해야 하며, 하드웨어 외부에서도 별도의 키 입력이나 무선 컨트롤로 수정이 가능해야 합니다.

## 개발 계획
이 프로젝트는 단계별로 업그레이드하여 더욱 정밀한 균형 제어를 목표로 합니다:

### 1단계: 1축 균형 장치
- **목표**: 한 축 방향의 균형 제어
- **구성 요소**:
  - 자이로 센서
  - 밸런스 휠 모터 1개
  - 마이크로컨트롤러 (Arduino 기반) 혹은 마이크로파이썬 보드
- **기능**: 기본적인 균형 유지 및 제어 알고리즘 구현

### 2단계: 2축 균형 장치  
- **목표**: 두 축 방향의 균형 제어


### 3단계: 3축 균형 장치
- **목표**: 완전한 3차원 균형 제어


## 프로젝트 문서
- 📋 **[제작 계획서](documents/제작계획서.md)** - 상세한 개발 일정 및 진행 계획
- 🔧 **하드웨어 우선 설계** - 기계 구조 및 전자 회로 설계부터 시작

## 디렉토리 구조
- `arduino/` - 아두이노 코드 및 펌웨어
- `hardware/` - 하드웨어 설계 파일
- `documents/` - 프로젝트 문서 및 설계서
  - `제작계획서.md` - 상세 개발 계획 및 일정
- `easyEDA/` - 회로 설계 파일



<<<<<<< HEAD






https://oshwhub.com/iMcHineSe/mini_simplefoc
https://oshwhub.com/flowersauce/simplefoc4008
https://oshwhub.com/flowersauce/drv8313

https://oshwhub.com/yourallo/youngfoc


드라이버는 여러가지 고민 중 심플하게 simpleFOC 드라이버로 진행. 
모터는 RC용 BLDC 모터를 쓸 수 도 있지만
기존에 구입했던 저렴이  BLDC 모터에 simpleFOC 드라이버를 연결.

이경우 simplefoc 드라이버에 신호를 주려면 in1, in2,in3  3pwm 필요하며, 
en, nfault, nsleep, nreset 핀 연결도 필요

모터가 총 3개가 될 예정이므로 디지털 io 핀이 6*3 18 + en 핀 추가로 19개 필요하게 된다. 
추가로 자기각 센서 등도 고려하므로 spi/i2c 핀도 고려해야 함. 

WeAct-STM32G431C 보드나, WeAct STM32F103VET6 보드를 고려중. 

simpleFOC 센서 
https://oshwhub.com/flowersauce/simplefoc4008

https://oshwhub.com/flowersauce/drv8313

전류 센싱 회로 참조
https://oshwhub.com/expert/bu-zu-bai-yuan-de-simplefoc-wu-shua-dian-ji-qu-dong-ban

여기서는 drv8313 을 사용했는데, 외부 fet를 사용하는 건.. 힘들것 같다

dengfoc 도 괜찮을듯?

simpleFOC 센서 
https://oshwhub.com/flowersauce/simplefoc4008

https://oshwhub.com/flowersauce/drv8313

전류 센싱 회로 참조
https://oshwhub.com/expert/bu-zu-bai-yuan-de-simplefoc-wu-shua-dian-ji-qu-dong-ban

여기서는 drv8313 을 사용했는데, 외부 fet를 사용하는 건.. 힘들것 같다

dengfoc 도 괜찮을듯?


=======
simpleFOC 센서 
https://oshwhub.com/flowersauce/simplefoc4008

https://oshwhub.com/flowersauce/drv8313

전류 센싱 회로 참조
https://oshwhub.com/expert/bu-zu-bai-yuan-de-simplefoc-wu-shua-dian-ji-qu-dong-ban

여기서는 drv8313 을 사용했는데, 외부 fet를 사용하는 건.. 힘들것 같다

dengfoc 도 괜찮을듯?



>>>>>>> 2d2ac7cffcb6432238161427eade4c15257a1bcf

