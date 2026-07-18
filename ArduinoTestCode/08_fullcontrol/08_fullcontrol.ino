// SimpleFOC 전체 제어모드 테스트 (MKS ESP32 FOC Mega 보드)
// AS5047P(SPI) 센서 + 보드 내장 전류센서(INA240 x2)를 모두 연결하여
// 개루프/폐루프를 포함한 SimpleFOC의 모든 MotionControlType / TorqueControlType을
// 시리얼 명령만으로 재부팅 없이 전환해가며 하나씩 테스트할 수 있습니다.
//
// 06_stepcommutation 자동 진단 테스트에서 확인된 값 사용:
//   - 극쌍수(PP) 추정치: 5 (실측, 필요 시 자석 개수로 재확인 권장)
//   - 회전 방향: 스텝 증가 방향과 실제 회전 방향 일치 (정방향)
//
// ===== 시리얼 명령 사용법 (SimpleFOC Commander 프로토콜) =====
//  MC0 : 모드 = torque             (전압/전류 기반 토크 제어)
//  MC1 : 모드 = velocity           (폐루프 속도 제어, 센서 필요)
//  MC2 : 모드 = angle              (폐루프 위치 제어, 센서 필요)
//  MC3 : 모드 = velocity_openloop  (개루프 속도 제어, 센서 불필요)
//  MC4 : 모드 = angle_openloop     (개루프 위치 제어, 센서 불필요)
//
//  MT0 : 토크 제어방식 = voltage      (전압 기반, 기본값)
//  MT1 : 토크 제어방식 = dc_current   (전류 크기 기반, 전류센서 필요)
//  MT2 : 토크 제어방식 = foc_current  (d,q축 전류 기반, 전류센서 필요)
//
//  M[숫자]   : 목표값 설정 (모드에 따라 전압[V]/속도[rad/s]/각도[rad] 중 하나로 해석됨, 예: M2, M-1.5)
//  ME0/ME1  : 모터 disable / enable
//  MLU[값]  : voltage_limit 설정 (예: MLU5)
//  MLC[값]  : current_limit 설정 (예: MLC1)
//  MLV[값]  : velocity_limit 설정 (예: MLV20)
//  MV,MA,MQ,MD : 속도/각도/전류q/전류d PID 게인 조회·설정 (예: MVP0.5)
//
// 자세한 명령 체계: docs.simplefoc.com/commander_interface

#include <SimpleFOC.h>

// ===== 핀 정의 (MKS ESP32 FOC Mega 보드 실측) =====
#define PIN_A 32
#define PIN_B 33
#define PIN_C 25
#define PIN_EN 12

#define PIN_CS0_A 39  // A상 전류센서 아날로그 입력 (SENSOR_VN)
#define PIN_CS0_B 36  // B상 전류센서 아날로그 입력 (SENSOR_VP)

#define PIN_SPI_SCLK 18
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 23
#define PIN_SPI_CS   5

// ===== 객체 =====
BLDCMotor motor = BLDCMotor(7); // 극쌍수 5 (06_stepcommutation 자동측정 추정치)
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_A, PIN_B, PIN_C, PIN_EN);
InlineCurrentSense current_sense = InlineCurrentSense(500.0f, PIN_CS0_A, PIN_CS0_B); // INA240A2(50V/V) x 10mΩ = 500mV/A
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, PIN_SPI_CS);

// 시리얼 커맨더 (SimpleFOC 표준 motor() 프로토콜 전체 사용)
Commander command = Commander(Serial);
void onMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // ---- 센서 초기화 ----
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);
  sensor.init();
  motor.linkSensor(&sensor);

  // ---- 드라이버 초기화 ----
  driver.voltage_power_supply = 9.0;
  driver.voltage_limit = 5.0;
  if (!driver.init()) {
    Serial.println("드라이버 초기화 실패!");
    return;
  }
  motor.linkDriver(&driver);

  // ---- 전류센서 초기화 (드라이버 disable 상태에서 오프셋 자동 캘리브레이션) ----
  current_sense.linkDriver(&driver);
  if (!current_sense.init()) {
    Serial.println("전류센서 초기화 실패!");
    return;
  }
  current_sense.skip_align = true;
  motor.linkCurrentSense(&current_sense);

  // ---- 안전 기본값 (필요 시 시리얼 명령으로 상향 조정) ----
  motor.voltage_limit = 5.0;      // MLU 로 변경 가능
  motor.current_limit = 1.0;      // MLC 로 변경 가능 (dc_current/foc_current 모드에서 사용)
  motor.velocity_limit = 20.0;    // MLV 로 변경 가능

  // 속도 PID (초기 튜닝값, 필요 시 MVP/MVI/MVD 로 조정)
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 2.0f;
  motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = 0.01f;

  // 각도 P 게인 (필요 시 MAP 로 조정)
  motor.P_angle.P = 20;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // 시작 모드: 가장 안전한 개루프 속도 제어부터 (센서 정렬 문제와 무관하게 항상 동작)
  motor.controller = MotionControlType::velocity_openloop;
  motor.torque_controller = TorqueControlType::voltage;

  motor.useMonitoring(Serial);
  motor.monitor_downsample = 200; // 시리얼 스팸 방지 (약 200 루프마다 1회 출력)

  motor.init();
  motor.initFOC(); // 센서 정렬 + 영점 전기각 탐색 (폐루프 모드 전환 시 필요)

  motor.target = 0; // 초기 목표값 0 (안전)

  command.add('M', onMotor, "motor");

  Serial.println(F("=== SimpleFOC 전체 제어모드 테스트 준비 완료 ==="));
  Serial.println(F("기본 모드: velocity_openloop, target=0"));
  Serial.println(F("예) M5      -> 목표 5 rad/s 로 개루프 회전"));
  Serial.println(F("    MC1     -> 폐루프 velocity 모드로 전환"));
  Serial.println(F("    M3      -> (velocity 모드에서) 목표 3 rad/s"));
  Serial.println(F("    MC2     -> 폐루프 angle 모드로 전환"));
  Serial.println(F("    M3.14   -> (angle 모드에서) 목표 각도 3.14 rad"));
  Serial.println(F("    MT2     -> 토크제어를 foc_current 로 전환 후 MC0, M0.3 등으로 테스트"));
  Serial.println(F("    ME0     -> 모터 즉시 정지(disable)"));

  _delay(1000);
}

void loop() {
  motor.loopFOC();
  motor.move();
  command.run();
}
