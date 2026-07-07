// MKS ESP32 FOC 개루프 위치 제어 예제 | 테스트 라이브러리: SimpleFOC 2.1.1 | 테스트 하드웨어: MKS ESP32 FOC V1.0
// 시리얼 포트에서 "T+숫자"를 입력해 두 모터의 위치를 설정합니다. 예: 180도로 회전 → "T3.14" 입력 (라디안 단위의 180도)
// 사용하는 모터에 맞게 기본 극 쌍 수를 수정하세요. 즉, BLDCMotor(7)의 값을 자신의 모터 극 쌍 수로 변경하세요.
// 프로그램 기본 공급 전압은 12V입니다. 다른 전압 사용 시 voltage_power_supply, voltage_limit 변수 값을 수정하세요.

#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);         //사용하는 모터에 맞게 올바른 극 쌍 수를 수정하세요. 즉, BLDCMotor()의 값을 변경하세요.
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor1 = BLDCMotor(7);        //마찬가지로 여기의 극 쌍 수 값을 수정하세요.
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

//목표 변수
float target_velocity = 0;

//시리얼 명령 설정
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {

  driver.voltage_power_supply = 12;       //모터 공급 전압에 맞게 voltage_power_supply 값을 수정하세요.
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_limit = 12;   // [V]      //모터 공급 전압에 맞게 voltage_limit 값을 수정하세요.
  motor.velocity_limit = 15; // [rad/s]
  
  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.voltage_limit = 12;   // [V]
  motor1.velocity_limit = 15; // [rad/s]

 
  //개루프 제어 모드 설정
  motor.controller = MotionControlType::angle_openloop;
  motor1.controller = MotionControlType::angle_openloop;

  //하드웨어 초기화
  motor.init();
  motor1.init();


  //T 명령 추가
  //시리얼 모니터에서 "T+숫자" 명령을 입력하고 전송하면 두 모터를 지정한 위치로 회전시킬 수 있습니다.
  //예: T3.14는 정방향 180° 위치로 회전
  command.add('T', doTarget, "target velocity");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
  motor.move(target_velocity);
  motor1.move(target_velocity);

  //사용자 통신
  command.run();
}
