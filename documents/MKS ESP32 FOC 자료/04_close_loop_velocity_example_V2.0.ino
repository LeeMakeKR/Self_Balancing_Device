/*
MKS ESP32 FOC 폐루프 속도 제어 예제 | 테스트 라이브러리: SimpleFOC 2.1.1 | 테스트 하드웨어: MKS ESP32 FOC V1.0
시리얼 창에서 T+속도를 입력하면 두 모터가 폐루프로 회전합니다.
예: 두 모터를 10rad/s로 회전시키려면 T10을 입력하세요.
사용하는 모터에 맞게 기본 극 쌍 수를 수정하세요. 즉, BLDCMotor(7)의 값을 자신의 모터 극 쌍 수로 변경하세요.
프로그램 기본 공급 전압은 12V입니다. 다른 전압 사용 시 voltage_power_supply, voltage_limit 변수 값을 수정하세요.
기본 PID는 2804 짐벌 모터를 기준으로 합니다. 다른 모터 사용 시 PID 파라미터를 수정해야 더 좋은 결과를 얻을 수 있습니다.
 */
#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

//모터 파라미터
BLDCMotor motor = BLDCMotor(7);                           //다른 모터 사용 시 극 쌍 수에 맞게 BLDMotor() 값을 수정하세요.
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor1 = BLDCMotor(7);                          //마찬가지로 여기의 BLDMotor() 값을 수정하세요.
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

//명령 설정
float target_velocity = 0;                                          //시리얼 창에서 T+속도를 입력하면 두 모터가 폐루프로 회전합니다.
Commander command = Commander(Serial);                              //예: 두 모터를 10rad/s로 회전시키려면 T10을 입력하세요.
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  I2Cone.begin(19, 18, 400000); 
  I2Ctwo.begin(23, 5, 400000);
  sensor.init(&I2Cone); 
  sensor1.init(&I2Ctwo);
  //motor 객체와 센서 객체 연결
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  //공급 전압 설정 [V]
  driver.voltage_power_supply = 12;               //다른 공급 전압 사용 시 voltage_power_supply 값을 수정하세요.
  driver.init();

  driver1.voltage_power_supply = 12;              //마찬가지로 voltage_power_supply 값을 수정하세요.
  driver1.init();
  //모터와 driver 객체 연결
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);
  
  //FOC 모드 선택
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //운동 제어 모드 설정
  motor.controller = MotionControlType::velocity;
  motor1.controller = MotionControlType::velocity;


  //속도 PI 루프 설정
  motor.PID_velocity.P = 0.1;
  motor1.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 1;
  motor1.PID_velocity.I = 1;
  motor.PID_velocity.D = 0;
  motor1.PID_velocity.D = 0;
  //최대 모터 전압 제한
  motor.voltage_limit = 12;                   //다른 공급 전압 사용 시 voltage_limit 값을 수정하세요.
  motor1.voltage_limit = 12;                  //마찬가지로 voltage_limit 값을 수정하세요.
  
  //속도 저역통과 필터 시정수
  motor.LPF_velocity.Tf = 0.01;
  motor1.LPF_velocity.Tf = 0.01;

  //최대 속도 제한 설정
  motor.velocity_limit = 40;
  motor1.velocity_limit = 40;

  Serial.begin(115200);
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);
  
  //모터 초기화
  motor.init();
  motor1.init();
  //FOC 초기화
  motor.initFOC();
  motor1.initFOC();
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  
}



void loop() {
  motor.loopFOC();
  motor1.loopFOC();

  motor.move(target_velocity);
  motor1.move(target_velocity);

  command.run();
}
