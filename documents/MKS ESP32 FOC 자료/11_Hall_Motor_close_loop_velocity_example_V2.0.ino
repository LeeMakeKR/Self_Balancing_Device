  /**
MKS ESP32 FOC 홀 센서 폐루프 속도 제어 예제 | 테스트 라이브러리: SimpleFOC 2.2.1 | 테스트 하드웨어: MKS ESP32 FOC V1.0
시리얼 창에서 T+속도를 입력하면 두 모터가 폐루프로 회전합니다.
예: 두 모터를 20rad/s로 회전시키려면 T20을 입력하세요.
사용하는 모터에 맞게 기본 극 쌍 수를 수정하세요. 즉, BLDCMotor()와 HallSensor()의 값을 자신의 모터 극 쌍 수로 변경하세요.
프로그램 기본 공급 전압은 24V입니다. 다른 전압 사용 시 voltage_power_supply, voltage_limit 변수 값을 수정하세요.
PID 파라미터는 실제 상황에 맞게 직접 조정하세요.
*/

#include <SimpleFOC.h>
//18——보드의 SCL_0 핀에 해당
//19——보드의 SDA_0 핀에 해당
//15——보드의 I_0 핀에 해당
//1——극 쌍 수
HallSensor sensor = HallSensor(18, 19, 15, 1);// U V W 극 쌍 수
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}
//5——보드의 SCL_0 핀에 해당
//23——보드의 SDA_0 핀에 해당
//13——보드의 I_0 핀에 해당
//1——극 쌍 수
HallSensor sensor1 = HallSensor(5, 23, 13, 1); // U V W 극 쌍 수
void doA1(){sensor1.handleA();}
void doB1(){sensor1.handleB();}
void doC1(){sensor1.handleC();}

//모터 파라미터 - 모터에 맞게 극 쌍 수 설정
BLDCMotor motor = BLDCMotor(1);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor1 = BLDCMotor(1);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 21);

//명령 설정
float target_velocity = 5;
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  sensor.init();
  sensor1.init();
  sensor.enableInterrupts(doA, doB, doC);
  sensor1.enableInterrupts(doA1, doB1, doC1);

  
  //motor 객체와 센서 객체 연결
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  //공급 전압 설정 [V]
  driver.voltage_power_supply = 24;
  driver.init();

  driver1.voltage_power_supply = 24;
  driver1.init();
  //모터와 driver 객체 연결
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);

  // aligning voltage [V]
  motor.voltage_sensor_align = 3;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;
  
  //운동 제어 모드 설정
  motor.controller = MotionControlType::velocity;
  motor1.controller = MotionControlType::velocity;

  //속도 PI 루프 설정
  motor.PID_velocity.P = 0.01;
  motor1.PID_velocity.P = 0.01;
  motor.PID_velocity.I = 0.1;
  motor1.PID_velocity.I = 0.1;
  motor.PID_velocity.D = 0;
  motor1.PID_velocity.D = 0;
  //각도 P 루프 설정 
  motor.P_angle.P = 20;
  motor1.P_angle.P = 20;
  //최대 모터 전압 제한
  motor.voltage_limit = 6;
  motor1.voltage_limit = 6;

  motor.PID_velocity.output_ramp = 1000;
  motor1.PID_velocity.output_ramp = 1000;
  
  //속도 저역통과 필터 시정수
  motor.LPF_velocity.Tf = 0.01f;
  motor1.LPF_velocity.Tf = 0.01f;

  //최대 속도 제한 설정
  motor.velocity_limit = 45;
  motor1.velocity_limit = 45;

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
//  sensor.update();
//  sensor1.update();

//  Serial.print(sensor1.getAngle());
//  Serial.print("\t");
//  Serial.println(sensor1.getVelocity());
}
