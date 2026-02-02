# Self_Balancing_Device

## 프로젝트 개요


### 모터 시스템
- **BLDC 모터** 사용

  - **컨트롤러 비내장형**: SimpleFOC 컨트롤러(DRV8313)와 자기각 센서(AS5600) 사용

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





simpleFOC 센서 
https://oshwhub.com/flowersauce/simplefoc4008

https://oshwhub.com/flowersauce/drv8313

전류 센싱 회로 참조
https://oshwhub.com/expert/bu-zu-bai-yuan-de-simplefoc-wu-shua-dian-ji-qu-dong-ban

여기서는 drv8313 을 사용했는데, 외부 fet를 사용하는 건.. 힘들것 같다




mcu  선정
축마다 esp32 개별보드로 제작한다
추후 다른 프로젝트에도 쉽게 사용 가능하고, 개발이 비교적 쉬움

이 경우 메인 컨트롤 보드가 별도로 필요. 
메인 보드와 서브 보드간 통신은? 프로토콜은?

