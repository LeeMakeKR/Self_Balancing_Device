/**
MKS ESP32 FOC 모터 노브 예제 | 테스트 라이브러리: SimpleFOC 2.1.1 | 테스트 하드웨어: MKS ESP32 FOC
전원 인가 후 한 모터를 회전시키면 다른 모터의 속도를 조절할 수 있습니다.
사용하는 모터에 맞게 기본 극 쌍 수를 수정하세요. 즉, BLDCMotor(7)의 값을 자신의 모터 극 쌍 수로 변경하세요.
프로그램 기본 공급 전압은 12V입니다. 다른 전압 사용 시 voltage_power_supply, voltage_limit 변수 값을 수정하세요.
기본 PID는 YT2804 모터를 기준으로 하며, 엔코더는 AS5600을 사용합니다. 다른 모터 사용 시 PID 파라미터를 수정해야 더 좋은 결과를 얻을 수 있습니다.
 */
#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

// 모터 상세 정보
BLDCMotor motor = BLDCMotor(7);                         //사용하는 모터의 극 쌍 수에 맞게 BLDCMotor() 값을 수정하세요.
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor1 = BLDCMotor(7);                        //마찬가지로 여기의 BLDCMotor() 값을 수정하세요.
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

//TROT 보행 변수 정의
void setup() {
  I2Cone.begin(19, 18, 400000); 
  I2Ctwo.begin(23, 5, 400000);
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  // 드라이버 설정
  // 공급 전압 [V]
  driver.voltage_power_supply = 12;               //공급 전압에 맞게 voltage_power_supply 값을 수정하세요.
  driver.init();

  driver1.voltage_power_supply = 12;              //마찬가지로 voltage_power_supply 값을 수정하세요.
  driver1.init();
  // 드라이버 연결
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);
  
  // 제어 루프 모드 설정
  motor.controller = MotionControlType::torque;
  motor1.controller = MotionControlType::velocity;

  // 모터 전압 제한
  motor.voltage_limit = 12;                 //공급 전압에 맞게 voltage_limit 값을 수정하세요.
  motor1.voltage_limit = 12;                //마찬가지로 voltage_limit 값을 수정하세요.
  
  motor1.LPF_velocity.Tf = 0.01;            //사용하는 모터에 맞게 이 두 파라미터를 수정하면 더 좋은 결과를 얻을 수 있습니다.
  motor1.PID_velocity.I = 1;

  // 시리얼 설정
  Serial.begin(115200);
  // 아래 두 줄이 필요하지 않으면 주석 처리하세요.
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);
  //초기 위치 기록

  
  //모터 초기화
  motor.init();
  motor1.init();
  motor.initFOC();
  motor1.initFOC();


  Serial.println("Motor ready.");
  _delay(1000);
  
}

void loop() {

  motor.loopFOC();
  motor1.loopFOC();

  motor.move(5*(motor1.shaft_velocity/10 - motor.shaft_angle));
  motor1.move(10*dead_zone(motor.shaft_angle));
}

float dead_zone(float x){
  return abs(x) < 0.2 ? 0 : x;
}
