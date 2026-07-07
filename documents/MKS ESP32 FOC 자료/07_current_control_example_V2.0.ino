
/**
MKS ESP32 FOC | FOC 전류 제어 예제 | 테스트 라이브러리: SimpleFOC 2.1.1 | 테스트 하드웨어: MKS ESP32 FOC V1.0
시리얼 창에서 A+전류로 M0 제어, B+전류로 M1 제어합니다. 전류 단위는 A(암페어)입니다.
코드에서 설정한 전압 제한 및 전류 제한을 변경하거나 주석 처리하여 비활성화할 수 있습니다.
사용하는 모터에 맞게 기본 극 쌍 수를 수정하세요. 즉, BLDCMotor(7)의 값을 자신의 모터 극 쌍 수로 변경하세요.
프로그램 기본 공급 전압은 12V입니다. 다른 전압 사용 시 voltage_power_supply, voltage_limit 변수 값을 수정하세요.
기본 PID는 YT2804 모터를 기준으로 합니다. 다른 모터 사용 시 PID 파라미터를 수정해야 더 좋은 결과를 얻을 수 있습니다.
*/

#include <SimpleFOC.h>

//모터 인스턴스
BLDCMotor motor1 = BLDCMotor(7);                        //사용하는 모터의 극 쌍 수에 맞게 BLCDMotor() 값을 수정하세요.
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32,33,25,22);

BLDCMotor motor2 = BLDCMotor(7);                        //마찬가지로 여기의 BLCDMotor() 값을 수정하세요.
BLDCDriver3PWM driver2  = BLDCDriver3PWM(26,27,14,12);

//엔코더 인스턴스
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Ctwo = TwoWire(1);


// 인라인 전류 감지 인스턴스
InlineCurrentSense current_sense1 = InlineCurrentSense(0.01, 50.0, 39, 36);
InlineCurrentSense current_sense2 = InlineCurrentSense(0.01, 50.0, 35, 34);

// commander 통신 인스턴스
Commander command = Commander(Serial);
void doMotor1(char* cmd){ command.motor(&motor1, cmd); }
void doMotor2(char* cmd){ command.motor(&motor2, cmd); }

void setup() {
  // 엔코더 설정
  I2Cone.begin(19,18, 400000UL); 
  I2Ctwo.begin(23,5, 400000UL); 
  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);
  

  // 드라이버 설정
  driver1.voltage_power_supply = 12;            //공급 전압에 맞게 voltage_power_supply 값을 수정하세요.
  driver1.init();
  motor1.linkDriver(&driver1);
  driver2.voltage_power_supply = 12;            //마찬가지로 voltage_power_supply 값을 수정하세요.
  driver2.init();
  motor2.linkDriver(&driver2);

  // 전류 제한
    motor1.current_limit = 2;         //상황에 맞게 전압/전류 제한을 수정하거나, 이 코드를 주석 처리하여 비활성화할 수 있습니다.
    motor2.current_limit = 2;
  // 전압 제한
    motor1.voltage_limit = 12;        //공급 전압에 맞게 voltage_limit 값을 수정하세요.
    motor2.voltage_limit = 12;


  // 전류 감지
  current_sense1.init();
  current_sense1.gain_b *= -1;
  current_sense1.gain_a *= -1;
//  current_sense1.skip_align = true;
  motor1.linkCurrentSense(&current_sense1);
  // current sense init and linking
  current_sense2.init();
  current_sense2.gain_b *= -1;
  current_sense2.gain_a *= -1;
//  current_sense2.skip_align = true;
  motor2.linkCurrentSense(&current_sense2);

  // 제어 루프
  // 다른 모드: TorqueControlType::voltage TorqueControlType::dc_current 
  motor1.torque_controller = TorqueControlType::foc_current; 
  motor1.controller = MotionControlType::torque;
  motor2.torque_controller = TorqueControlType::foc_current; 
  motor2.controller = MotionControlType::torque;

  // FOC 전류 제어 PID 파라미터
   motor1.PID_current_q.P = 2;                      //더 좋은 결과를 위해 적절한 PID 파라미터를 조정하세요.
   motor1.PID_current_q.I= 800;                     //모터가 떨리거나 회전 속도가 불안정하다면 PID 파라미터가 적절히 조정되지 않은 것일 수 있습니다.
   motor1.PID_current_d.P= 2;
   motor1.PID_current_d.I = 800;
   motor1.LPF_current_q.Tf = 0.002; // 1ms default
   motor1.LPF_current_d.Tf = 0.002; // 1ms default

   motor2.PID_current_q.P = 2;
   motor2.PID_current_q.I= 800;
   motor2.PID_current_d.P= 2;
   motor2.PID_current_d.I = 800;
   motor2.LPF_current_q.Tf = 0.002; // 1ms default
   motor2.LPF_current_d.Tf = 0.002; // 1ms default

    // 속도 루프 PID 파라미터
    motor1.PID_velocity.P = 0.1;
    motor1.PID_velocity.I = 1;
    motor1.PID_velocity.D = 0;

    motor2.PID_velocity.P = 0.1;
    motor2.PID_velocity.I = 1;
    motor2.PID_velocity.D = 0;
    // default voltage_power_supply
  
    // 속도 제한
    motor1.velocity_limit = 40;
    motor2.velocity_limit = 40;


  // monitor 인터페이스 설정
  Serial.begin(115200);
  // comment out if not needed
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  // monitor 관련 설정
  motor1.monitor_downsample = 0;
  motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;
  motor2.monitor_downsample = 0;
  motor2.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;
  


  //모터 초기화
  motor1.init();
  // align encoder and start FOC
  motor1.initFOC(); 
  
  motor2.init();
  // align encoder and start FOC
  motor2.initFOC(); 

  // 초기 목표값
  motor1.target = 0.05;
  motor2.target = 0.05;

  // 모터를 commander에 매핑
  command.add('A', doMotor1, "motor 1");
  command.add('B', doMotor2, "motor 2");

  Serial.println(F("Double motor sketch ready."));
  
  _delay(1000);
}


void loop() {
  // iterative setting FOC phase voltage
  motor1.loopFOC();
  motor2.loopFOC();

  // iterative function setting the outter loop target
  motor1.move();
  motor2.move();

  // user communication
  command.run();
  motor1.monitor();
  motor2.monitor();
}
