#include <SimpleFOC.h>

// BLDCDriver3PWM(pwmA, pwmB, pwmC, en) 핀 지정 반영
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 1, 7, 4);

void setup() {
  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  SimpleFOCDebug::enable(&Serial);
  
  // 파워 서플라이 공급 전압 (사용자 전원에 맞게 수정 가능)
  driver.voltage_power_supply = 12.0;
  // 초기 테스트용 안전 제한 전압 (1V 설정으로 모터 발열 방지)
  driver.voltage_limit = 1.0;

  // driver init
  if (!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }

  // enable driver
  driver.enable();
  Serial.println("Driver ready!");
  _delay(1000);
}

void loop() {
    // 제어 전압이 voltage_limit(1.0V) 범위 내로 제한되도록 안전값 출력
    driver.setPwm(0, 0, 1);
    delay(1000);
    driver.setPwm(0, 1, 0);
    delay(1000);
    driver.setPwm(1, 0, 0);
    delay(1000);

}