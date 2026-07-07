#include <SimpleFOC.h>

// BLDCMotor(극쌍수 pole pair number) 설정
BLDCMotor motor = BLDCMotor(7);
// BLDCDriver3PWM(pwmA, pwmB, pwmC, en) 핀 지정 반영
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 1, 7, 4);

// AS5600 I2C 자기식 센서 객체 생성
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// 시리얼 커맨더(Commander) 객체 생성
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() { 
  // 시리얼 모니터 포트 시작
  Serial.begin(115200);
  
  // 디버깅용 상세 출력 활성화
  SimpleFOCDebug::enable(&Serial);
  
  // ESP32-C3 사용자 지정 I2C 핀 초기화 (SDA: 6, SCL: 5)
  Wire.begin(6, 5);
  // I2C 클록 속도 설정 (400kHz 고속 모드)
  Wire.setClock(400000);

  // 센서 하드웨어 초기화 및 모터에 센서 연결
  sensor.init();
  motor.linkSensor(&sensor);

  // 드라이버 설정
  driver.voltage_power_supply = 12.0; // 파워 서플라이 공급 전압 [V]
  driver.voltage_limit = 1.0;         // 드라이버 출력 최대 허용 전압 제한 (안전값 1V)
  
  // 드라이버 초기화 및 모터 연결
  if(!driver.init()){
    Serial.println("드라이버 초기화 실패!");
    return;
  }
  motor.linkDriver(&driver);

  // 센서 자석과 모터 위상 정렬 시 인가할 전압 (과전류 방지용 1V 설정)
  motor.voltage_sensor_align = 1.0;

  // 제어 루프 설정: 전압 기반 토크 제어 모드
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  // 모니터링 기능 활성화
  motor.useMonitoring(Serial);

  // 모터 하드웨어 초기화
  if(!motor.init()){
    Serial.println("모터 초기화 실패!");
    return;
  }
  // 센서 전기적 영점 정렬 및 FOC 제어 시작
  if(!motor.initFOC()){
    Serial.println("FOC 초기화 실패!");
    return;
  }

  // 초기 목표 제어 전압 설정 [V]
  motor.target = 0.2; 

  // 시리얼 명령 입력 인터페이스 추가 (M: 모터 제어 명령)
  command.add('M', doMotor, "Motor");

  Serial.println(F("모터 및 I2C 센서 준비 완료."));
  _delay(1000);
}

void loop() {
  // FOC 알고리즘 루프 (최대한 빠르게 반복 호출 실행 필요)
  motor.loopFOC();

  // 모션 제어 구동 수행
  motor.move();

  // 사용자 시리얼 커맨드 처리 실행
  command.run();
}