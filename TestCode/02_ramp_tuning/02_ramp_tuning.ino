// 속도 상승률(램프율) 튜닝 테스트 (MKS ESP32 FOC Mega + AS5047P)
//
// 목적: 목표 RPM까지 "얼마나 빠르게" 올릴 수 있는지 찾는 것.
// 재업로드 없이 시리얼로 값을 바꿔가며 도달 시간을 반복 측정합니다.
//
// 확인되는 것:
//   - 모터가 제대로 회전하는가  -> actual 값이 cmd를 따라오는가
//   - 센서가 제대로 읽히는가    -> angle 값이 계속 변하는가
//
// 시리얼 명령 (115200 baud, 줄 끝에 Enter):
//   t1500   목표 RPM = 1500
//   r800    램프율 = 800 RPM/s   (r0 = 램프 없이 즉시 스텝)
//   p0.1    속도 PID의 P
//   i1.0    속도 PID의 I
//   o50     output_ramp (출력 전압 변화율 제한 V/s)
//   v11     전압 상한
//   g       실행 (0부터 목표까지 램프, 도달 시간 측정)
//   s       정지 (드라이버 off, 자연 감속)
//   ?       현재 설정 출력
//
// 튜닝 방법: r을 낮은 값부터 올리며 g를 반복. 도달 시간이 더 안 줄거나
// actual이 cmd를 못 따라가기 시작하는 지점이 한계입니다.

#include <SimpleFOC.h>

// ===== 핀 정의 (MKS ESP32 FOC Mega 보드 실측) =====
#define PIN_A  32
#define PIN_B  33
#define PIN_C  25
#define PIN_EN 12

#define PIN_SPI_SCLK 18
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 23
#define PIN_SPI_CS    5

// ===== 모터 파라미터 =====
#define POLE_PAIRS 10   // initFOC의 "PP check" 경고가 뜨면 로그의 est. pp 값으로 교체

const float SUPPLY_VOLTAGE = 12.0f;
const float RPM_CEILING    = 6000.0f;  // 목표 RPM 입력 상한 (오타 방지용)

// ===== 객체 =====
BLDCMotor         motor  = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM    driver = BLDCDriver3PWM(PIN_A, PIN_B, PIN_C, PIN_EN);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, PIN_SPI_CS);

// ===== 튜닝 대상 값 (시리얼로 변경) =====
float target_rpm = 1000.0f;  // 목표 RPM
float ramp_rate  = 500.0f;   // RPM/s. 0이면 즉시 스텝
float volt_limit = 11.0f;    // 전압 상한

// ===== 런타임 상태 =====
bool          running    = false;
bool          reached    = false;
float         cmd_rpm    = 0;      // 실제로 모터에 주는 지령
unsigned long run_start  = 0;
unsigned long last_ms    = 0;
unsigned long last_print = 0;

// ===== 단위 변환 =====
float rpmToRps(float rpm) { return rpm * 2.0f * PI / 60.0f; }
float rpsToRpm(float rps) { return rps * 60.0f / (2.0f * PI); }

void printSettings() {
  Serial.println(F("--- settings ---"));
  Serial.print(F("t target_rpm  = ")); Serial.println(target_rpm, 0);
  Serial.print(F("r ramp_rate   = ")); Serial.print(ramp_rate, 0);   Serial.println(F(" RPM/s"));
  Serial.print(F("p PID.P       = ")); Serial.println(motor.PID_velocity.P, 3);
  Serial.print(F("i PID.I       = ")); Serial.println(motor.PID_velocity.I, 3);
  Serial.print(F("o output_ramp = ")); Serial.println(motor.PID_velocity.output_ramp, 0);
  Serial.print(F("v volt_limit  = ")); Serial.println(volt_limit, 1);
  Serial.println(F("g = go, s = stop"));
}

void startRun() {
  // PID 내부 적분항이 이전 실행에서 남아 있으면 시작하자마자 튑니다.
  // LPF는 내부 상태가 protected라 손댈 수 없지만, Tf=0.02s라 수십 ms면 수렴합니다.
  motor.PID_velocity.reset();

  cmd_rpm   = 0;
  reached   = false;
  run_start = millis();
  last_ms   = run_start;
  running   = true;
  motor.enable();

  Serial.println();
  Serial.print(F(">>> RUN  target=")); Serial.print(target_rpm, 0);
  Serial.print(F(" RPM  ramp="));      Serial.print(ramp_rate, 0);
  Serial.println(F(" RPM/s"));
  Serial.println(F("ms\tcmd\tactual\tangle_deg"));
}

void stopRun() {
  running = false;
  cmd_rpm = 0;
  // 회전 중 move(0)은 급제동이라 역토크가 큽니다. 드라이버를 끄고 자연 감속시킵니다.
  motor.disable();
  Serial.println(F(">>> STOP (driver off, coasting down)"));
}

// 한 줄 단위로 명령을 읽어 처리
void handleSerial() {
  static char buf[24];
  static uint8_t len = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\r') continue;
    if (c != '\n') {
      if (len < sizeof(buf) - 1) buf[len++] = c;
      continue;
    }

    buf[len] = '\0';
    len = 0;
    if (buf[0] == '\0') continue;

    char  cmd = buf[0];
    float val = atof(buf + 1);

    switch (cmd) {
      case 't':
        target_rpm = constrain(val, 0.0f, RPM_CEILING);
        Serial.print(F("target_rpm = ")); Serial.println(target_rpm, 0);
        break;
      case 'r':
        ramp_rate = val < 0 ? 0 : val;
        Serial.print(F("ramp_rate = ")); Serial.println(ramp_rate, 0);
        break;
      case 'p':
        motor.PID_velocity.P = val;
        Serial.print(F("PID.P = ")); Serial.println(val, 3);
        break;
      case 'i':
        motor.PID_velocity.I = val;
        Serial.print(F("PID.I = ")); Serial.println(val, 3);
        break;
      case 'o':
        motor.PID_velocity.output_ramp = val;
        Serial.print(F("output_ramp = ")); Serial.println(val, 0);
        break;
      case 'v':
        volt_limit = constrain(val, 0.0f, SUPPLY_VOLTAGE);
        motor.voltage_limit = volt_limit;
        motor.PID_velocity.limit = volt_limit;
        Serial.print(F("volt_limit = ")); Serial.println(volt_limit, 1);
        break;
      case 'g': startRun(); break;
      case 's':
      case 'x': stopRun();  break;
      case '?': printSettings(); break;
      default:
        Serial.println(F("unknown cmd. use: t r p i o v g s ?"));
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  SimpleFOCDebug::enable(&Serial);

  Serial.println(F("=========================================="));
  Serial.println(F(" Ramp Rate Tuning"));
  Serial.println(F("=========================================="));

  // --- 센서 ---
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);
  sensor.clock_speed = 10000000;  // 기본 1MHz는 고RPM에서 병목
  sensor.init();
  motor.linkSensor(&sensor);

  // --- 드라이버 ---
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = volt_limit;
  if (!driver.init()) {
    Serial.println(F("[FAIL] driver init"));
    while (1) delay(100);
  }
  motor.linkDriver(&driver);

  // --- 폐루프 속도제어 ---
  motor.controller        = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation    = FOCModulationType::SpaceVectorPWM;
  motor.voltage_limit     = volt_limit;
  motor.velocity_limit    = rpmToRps(RPM_CEILING);

  motor.PID_velocity.P           = 0.1f;
  motor.PID_velocity.I           = 1.0f;
  motor.PID_velocity.D           = 0.0f;
  motor.PID_velocity.output_ramp = 50.0f;
  motor.LPF_velocity.Tf          = 0.02f;

  motor.init();
  motor.initFOC();

  if (motor.motor_status != FOCMotorStatus::motor_ready) {
    Serial.println(F("[FAIL] initFOC. Check POLE_PAIRS and sensor wiring."));
    while (1) delay(100);
  }

  motor.disable();  // 명령을 받기 전까지는 꺼둠

  sensor.update();
  Serial.print(F("sensor angle at boot = "));
  Serial.print(sensor.getAngle() * 180.0f / PI, 1);
  Serial.println(F(" deg"));
  Serial.println();
  printSettings();

  last_ms = millis();
}

void loop() {
  handleSerial();
  motor.loopFOC();

  unsigned long now = millis();
  float dt = (now - last_ms) / 1000.0f;
  last_ms = now;

  if (!running) {
    // 정지 중에도 감속 상황을 볼 수 있게 회전이 남아 있으면 출력
    if (now - last_print >= 500) {
      last_print = now;
      sensor.update();
      float rpm = rpsToRpm(sensor.getVelocity());
      if (fabs(rpm) > 20.0f) {
        Serial.print(F("coasting... ")); Serial.print(fabs(rpm), 0); Serial.println(F(" RPM"));
      }
    }
    return;
  }

  // --- 지령 램프 ---
  if (ramp_rate <= 0) {
    cmd_rpm = target_rpm;                 // 램프 없음: 즉시 스텝
  } else if (cmd_rpm < target_rpm) {
    cmd_rpm += ramp_rate * dt;
    if (cmd_rpm > target_rpm) cmd_rpm = target_rpm;
  }

  motor.move(rpmToRps(cmd_rpm));

  float actual_rpm = rpsToRpm(motor.shaft_velocity);

  // --- 도달 시간 측정 ---
  if (!reached && target_rpm > 0 && fabs(actual_rpm) >= target_rpm * 0.95f) {
    reached = true;
    Serial.print(F(">>> reached 95% of target in "));
    Serial.print(now - run_start);
    Serial.println(F(" ms"));
  }

  // --- 텔레메트리 ---
  if (now - last_print >= 200) {
    last_print = now;
    Serial.print(now - run_start); Serial.print('\t');
    Serial.print(cmd_rpm, 0);      Serial.print('\t');
    Serial.print(actual_rpm, 0);   Serial.print('\t');
    Serial.println(sensor.getAngle() * 180.0f / PI, 0);
  }
}
