// ESP32 FOC 스테퍼 모터 개루프 속도 제어 예제 | 테스트 라이브러리: SimpleFOC 2.2.1 | 테스트 하드웨어: MKS ESP32 FOC V1.0
// 스테퍼 모터 드라이브 선 연결: A+:C0 A-:B0 B+:C1 B-:B1
// 시리얼 포트에서 "T+숫자"를 입력해 모터 회전 속도를 조절합니다. 예: 10rad/s로 회전 → "T10" 입력. 전원 인가 시 기본값은 5rad/s입니다.
// 사용하는 모터에 맞게 기본 극 쌍 수를 수정하세요. 즉, StepperMotor(50)의 값을 자신의 모터 극 쌍 수로 변경하세요.
// 프로그램 기본 공급 전압은 12V입니다. 다른 전압 사용 시 voltage_power_supply 변수 값을 수정하세요.

#include <SimpleFOC.h>

// Stepper motor instance
StepperMotor motor = StepperMotor(50);                //사용하는 모터의 극 쌍 수에 맞게 StepperMotor() 값을 수정하세요.
// Stepper driver instance
StepperDriver4PWM driver = StepperDriver4PWM(33, 32,26,27, 22, 12); 


// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }

void setup() {

  driver.pwm_frequency = 40000;

  driver.voltage_power_supply = 12;               //공급 전압에 맞게 voltage_power_supply 값을 수정하세요.
  driver.init();
  motor.linkDriver(&driver);

  //개루프 제어 모드 설정
  motor.controller = MotionControlType::velocity_openloop;

  // 시리얼 초기화
  Serial.begin(115200);
  motor.useMonitoring(Serial);

  //초기화
  motor.init();

  // 초기 목표값 설정
  motor.target = 5;

  // T 명령 추가
  command.add('T', onMotor, "motor");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  
  _delay(1000);
}


void loop() {
  motor.move();

  command.run();

}
