// 개루프 제어는 불가피하게 발열 현상을 야기하므로 이 예제를 1분 이상 연속 실행하지 마십시오.
// 과열은 모터 또는 드라이버 보드를 태울 수 있습니다!

#include <SimpleFOC.h>

// BLDC 모터 극쌍수(pole pairs) 설정: 7
BLDCMotor motor = BLDCMotor(7);

// 3PWM 드라이버 핀 설정: A=32, B=33, C=25, Enable=12 (MKS ESP32 FOC Mega 보드 기준)
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

// 시리얼 커맨더 객체 생성 (목표 속도를 런타임에 변경하기 위함)
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }

void setup() {
  // 시리얼 모니터 포트 시작
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // 드라이버 전원 전압 설정 (사용하시는 공급 전원에 맞춰 수정)
  driver.voltage_power_supply = 12.0;
  // 개루프 제어 시 과열 방지를 위한 낮은 전압 제한 (필요 시 1~5V 사이로 조정)
  driver.voltage_limit = 5.0;

  if (!driver.init()) {
    Serial.println("드라이버 초기화 실패!");
    return;
  }
  motor.linkDriver(&driver);

  // 드라이버와 동일한 전압 제한 적용
  motor.voltage_limit = 5.0;

  // 제어 모드: 개루프 속도 제어 (센서 없이 일정한 속도로 회전, loopFOC 불필요)
  motor.controller = MotionControlType::velocity_openloop;

  if (!motor.init()) {
    Serial.println("모터 초기화 실패!");
    return;
  }

  // 목표 속도 [rad/s] (정속 회전, 양수: 정방향 / 음수: 역방향)
  motor.target = 5.0;

  // 시리얼 명령 입력 인터페이스 추가 (T: 목표 속도 변경, 예: T10)
  command.add('T', doTarget, "target velocity");

  Serial.println(F("모터 준비 완료. 개루프 정속 회전 테스트를 시작합니다."));
  Serial.println(F("시리얼 모니터에 T[값] 형식으로 목표 속도(rad/s)를 입력하면 즉시 반영됩니다 (예: T10)."));
  _delay(1000);
}

void loop() {
  // 개루프 제어는 loopFOC() 없이 move()만 반복 호출하면 됩니다
  motor.move();

  // 시리얼 명령 처리 (목표 속도 실시간 변경)
  command.run();

  // 500ms 주기로 현재 목표 속도 출력
  static unsigned long last_print = 0;
  if (millis() - last_print > 500) {
    Serial.print("Target Velocity (rad/s): ");
    Serial.println(motor.target);
    last_print = millis();
  }
}
