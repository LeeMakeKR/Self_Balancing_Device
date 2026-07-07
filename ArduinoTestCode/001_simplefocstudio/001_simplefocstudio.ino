#include <SimpleFOC.h>

// BLDCMotor(극쌍수) 설정
BLDCMotor motor = BLDCMotor(5);
// BLDCDriver3PWM(pwmA, pwmB, pwmC, en) 핀 지정 반영
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 1, 7, 4);

// AS5600 I2C 자기식 센서 객체 생성
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// 시리얼 커맨더 객체 생성
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() { 
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);
  
  // [수정 확인] ESP32-C3 사용자 지정 I2C 핀 초기화 (SDA: 6, SCL: 5)
  Wire.begin(6, 5);
  Wire.setClock(400000);

  // 센서 및 모터 연결 (I2C 핀 활성화 직후 실행)
  sensor.init();
  motor.linkSensor(&sensor);

  // 드라이버 설정 및 제한 전압(보호장치) 인가
  driver.voltage_power_supply = 12.0;
  driver.voltage_limit = 1.0; // 초기 안전 전압 1V 제한
  if(!driver.init()){
    Serial.println("드라이버 초기화 실패!");
    return;
  }
  motor.linkDriver(&driver);

  // FOC 얼라인먼트 및 제어 전압 제한
  motor.voltage_limit = 1.0;
  motor.voltage_sensor_align = 1.0;

  // 제어 모드: 전압 기반 토크 제어 (Studio에서 변경 가능)
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  // SimpleFOCStudio 모니터링 기능 활성화
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 10; // 모니터링 다운샘플링 설정 (루프 부하 감소)

  // 모터 및 FOC 초기화
  if(!motor.init() || !motor.initFOC()){
    Serial.println("모터 또는 FOC 초기화 실패!");
    return;
  }

  // SimpleFOCStudio의 기기 명령 ID 'M'과 함수 매핑
  command.add('M', doMotor, "motor");

  Serial.println(F("모터 및 Studio 연동 준비 완료."));
  _delay(1000);
}

void loop() {
  // FOC 알고리즘 고속 루프
  motor.loopFOC();
  motor.move();

  // SimpleFOCStudio 실시간 데이터 송수신 및 모니터링
  motor.monitor();
  command.run();
}