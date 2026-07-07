// ④ 개루프 제어는 불가피하게 발열 현상을 야기하므로 이 예제를 1분 이상 실행하지 마십시오. 과열은 모터 또는 드라이버 보드를 태울 수 있습니다!

#include <SimpleFOC.h>

// BLDC 모터 극쌍수(pole pairs) 설정: 7
BLDCMotor motor = BLDCMotor(7);

// 3PWM 드라이버 핀 설정: A=32, B=33, C=25, Enable=12
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

void setup() {
  // 드라이버 전원 전압 설정 (사용하시는 공급 전원에 맞춰 수정해 주세요)
  driver.voltage_power_supply = 12;
  driver.init();
  
  // 모터와 드라이버 연결
  motor.linkDriver(&driver);

  // 전압 제한 설정 (개루프 제어 시 과열 방지를 위해 최대한 낮춥니다)
  motor.voltage_limit = 5.0;   // 2V 제한 (필요에 따라 1V ~ 3V 사이로 조정)
  
  // 제어 모드를 개루프 속도 제어로 설정
  motor.controller = MotionControlType::velocity_openloop;

  // 모터 초기화
  motor.init();

  Serial.begin(115200);
  Serial.println("Motor ready! Open-loop velocity control test.");
}

void loop() {
  // 시간에 따라 속도가 가변되도록 설정 (5.0 rad/s ~ 15.0 rad/s 사이를 부드럽게 왕복하며 계속 회전)
  float time = millis() / 1000.0;
  float target_velocity = 10.0 + 5.0 * sin(time);

  // 모터 구동 및 개루프 제어 루프 실행 (loopFOC는 필요하지 않습니다)
  motor.move(target_velocity);
  
  // 시리얼 모니터로 현재 목표 속도 출력 (500ms 주기)
  static unsigned long last_print = 0;
  if (millis() - last_print > 500) {
    Serial.print("Target Velocity (rad/s): ");
    Serial.println(target_velocity);
    last_print = millis();
  }
}
