// 회전방향 확인 + 최대 RPM 측정 테스트 (MKS ESP32 FOC Mega + AS5047P)
//
// 제작 로그 20260731 항목: "as5047을 이용하여 모터의 회전과 각도를 확인하고,
// 모터의 회전을 높여가면서 최대 rpm 확인" 을 자동으로 수행합니다.
//
// 3단계로 진행됩니다:
//   [1단계] 개루프 방향 확인
//       - 폐루프 전에 확인해야 의미가 있습니다. 폐루프는 initFOC이 센서 방향을
//         이미 보정한 뒤라 "지령 부호 = 센서 부호"가 항상 성립해 테스트가 무의미해집니다.
//       - 낮은 전압(2V)으로 정방향/역방향을 돌려 AS5047 각도가 어느 쪽으로 증가하는지 기록.
//   [2단계] initFOC 정렬
//       - SimpleFOC가 스스로 판정한 sensor_direction(CW/CCW)과 극쌍수(PP)를 출력.
//       - 1단계 결과와 교차 검증합니다.
//   [3단계] 속도 램프 → 최대 RPM 탐색
//       - 폐루프 속도제어로 목표 RPM을 단계적으로 올림.
//       - 실제 RPM이 목표를 못 따라오기 시작하면(전압/역기전력 한계 = 포화)
//         그 직전 실제 RPM을 최대 RPM으로 기록하고 종료.
//
// 토크 제어는 voltage 모드를 씁니다. 최대 RPM은 역기전력 대 공급전압의 싸움이므로
// current_limit이 끼면 전류 한계에 먼저 걸려 "진짜 최대 RPM"이 안 나옵니다.
// 전류 제한별 RPM 특성이 궁금하면 09_current_rpm_sweep 쪽을 쓰세요.
//
// 시리얼 모니터에서 'x' 입력 시 즉시 정지합니다. 115200 baud.

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
// ProDrone 4212 실측: initFOC의 PP check가 est. pp = 10.16 을 보고했습니다.
// (PP=7로 돌렸을 때 회전 없이 진동만 하던 원인)
// 재실행 후에도 "PP check: fail" 경고가 남으면, 그때 나온 est. pp 값으로 다시 고칩니다.
// 경고가 사라지면 그 값이 맞는 것입니다.
#define POLE_PAIRS 10

// ===== 전원 / 안전 한계 =====
// ※ 밸런스휠(자이로 휠) 장착 상태 기준으로 조정된 값입니다.
//   휠 관성이 붙으면 가속 토크 요구가 수십 배로 커지므로 모든 변화율을 낮췄습니다.
const float SUPPLY_VOLTAGE   = 12.0f;   // 실제 배터리/파워서플라이 전압에 맞출 것
const float VOLTAGE_LIMIT    = 11.0f;   // 램프 구간 인가전압 상한
const float OPENLOOP_VOLTAGE = 5.0f;    // 3.0에서는 휠이 전혀 안 돌아 상향. 발열 주시할 것
const float RPM_CEILING      = 6000.0f; // 안전 상한. 휠 장착 상태에서는 낮게 시작할 것

// ===== 1단계: 방향 테스트 파라미터 =====
// 개루프는 회전자 위치를 모르고 강제로 전기각을 돌리는 방식이라, 관성이 크면
// 지령 속도를 따라오지 못하고 즉시 동기를 잃습니다. 저속 + 완만한 램프가 필수입니다.
const float         DIR_TEST_RPM = 60.0f;   // 개루프 시험 속도 (300 -> 60)
const float         DIR_RAMP_RPM_PER_S = 10.0f; // 개루프 목표 상승률 (30 -> 10). 0->60rpm 까지 6초
                                                // 필요 토크 = 관성 x 각가속도. 상승률을 낮추는 것이
                                                // 전압을 올리는 것보다 안전한 1차 대책입니다.
const unsigned long DIR_HOLD_MS  = 2500;    // 목표 도달 후 유지 시간
const float         DIR_MIN_DEG  = 20.0f;   // 이보다 적게 움직이면 "회전 안 함"으로 판정

// ===== 코스트다운(관성 정지 대기) 파라미터 =====
// 휠은 move(0)으로 안 멈춥니다. 정지 벡터를 물리면 락드로터 상태가 되어 전류만 치솟습니다.
// 드라이버를 끄고 자연 감속을 기다립니다.
const float         COAST_STOP_RPM   = 15.0f;  // 이 아래로 떨어지면 정지로 간주
const unsigned long COAST_TIMEOUT_MS = 20000;  // 최대 대기 시간

// ===== 3단계: 램프 파라미터 =====
const float         RAMP_STEP_RPM = 300.0f;  // 계단 목표 증가 단위 (200 -> 100)
const float         RAMP_RATE_RPM_PER_S = 40.0f; // 계단 사이를 메우는 상승률. 토크 스파이크 방지
const unsigned long SETTLE_MS     = 2500;    // 목표 도달 후 정착 대기 (800 -> 2500)
const float         TRACK_ERR_RPM = 120.0f;  // 정착 후 오차가 이보다 크면 포화로 간주
const int           SATURATE_CONFIRM = 3;    // 연속 포화 확인 횟수 (노이즈 방지)

// ===== 객체 =====
BLDCMotor       motor    = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM  driver   = BLDCDriver3PWM(PIN_A, PIN_B, PIN_C, PIN_EN);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, PIN_SPI_CS);

// ===== 상태 머신 =====
// 주의: 이 enum은 반드시 첫 함수 정의보다 위에 있어야 합니다.
// Arduino IDE가 자동 생성하는 함수 프로토타입이 첫 함수 앞에 삽입되기 때문에,
// enum이 아래에 있으면 enterPhase(TestPhase) 프로토타입에서 타입을 못 찾습니다.
enum TestPhase {
  PH_DIR_FWD,     // 개루프 정방향
  PH_DIR_REV,     // 개루프 역방향
  PH_COAST,       // 드라이버 끄고 관성 정지 대기
  PH_FOC_INIT,    // initFOC 정렬
  PH_RAMP,        // 속도 램프
  PH_DONE,
  PH_ABORT
};
TestPhase phase = PH_DIR_FWD;
TestPhase coast_next = PH_DIR_REV;  // 코스트다운이 끝나면 넘어갈 단계

// ===== 단위 변환 =====
float rpmToRps(float rpm) { return rpm * 2.0f * PI / 60.0f; }
float rpsToRpm(float rps) { return rps * 60.0f / (2.0f * PI); }

unsigned long phase_start_ms = 0;
unsigned long last_step_ms   = 0;
unsigned long last_print_ms  = 0;
unsigned long last_ramp_ms   = 0;  // 램프 적분용 이전 시각

// 1단계 결과
float dir_fwd_delta_deg = 0;
float dir_rev_delta_deg = 0;
float phase_start_angle = 0;
float dir_cmd_rpm       = 0;  // 개루프 지령. 0에서 DIR_TEST_RPM까지 완만히 상승

// 3단계 결과
float step_target_rpm = 0;   // 계단 목표 (RAMP_STEP_RPM 단위로 상승)
float target_rpm     = 0;    // 실제 지령. step_target_rpm 쪽으로 완만히 이동
float best_actual_rpm = 0;
int   saturate_count = 0;

bool userAbortRequested() {
  while (Serial.available()) {
    if ((char)Serial.read() == 'x') return true;
  }
  return false;
}

// 센서를 한 번 갱신하고 누적 각도를 도(deg) 단위로 반환
float readAngleDeg() {
  sensor.update();
  return sensor.getAngle() * 180.0f / PI;
}

// 드라이버가 꺼진 상태(코스트다운)에서도 쓸 수 있는 센서 직독 속도
float readSensorRpm() {
  sensor.update();
  return rpsToRpm(sensor.getVelocity());
}

void enterPhase(TestPhase next) {
  phase = next;
  phase_start_ms = millis();
  last_ramp_ms   = phase_start_ms;
  phase_start_angle = readAngleDeg();
}

// 드라이버를 끄고 휠이 자연 감속으로 멈추기를 기다리는 단계로 진입
void startCoast(TestPhase next) {
  motor.disable();
  coast_next = next;
  Serial.println(F("Coasting down (driver off, waiting for wheel to stop)..."));
  enterPhase(PH_COAST);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  SimpleFOCDebug::enable(&Serial);

  Serial.println(F("=========================================="));
  Serial.println(F(" Rotation Direction Check + Max RPM Test"));
  Serial.println(F(" Send 'x' at any time to abort"));
  Serial.println(F("=========================================="));

  // --- 센서 초기화 ---
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);
  // 기본 1MHz로는 고RPM 구간에서 각도 읽기가 제어루프 병목이 됩니다. AS5047P 상한인 10MHz로 올림.
  sensor.clock_speed = 10000000;
  sensor.init();
  motor.linkSensor(&sensor);

  // --- 드라이버 초기화 ---
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = VOLTAGE_LIMIT;
  if (!driver.init()) {
    Serial.println(F("[FAIL] Driver init failed. Check power supply and pin wiring."));
    phase = PH_ABORT;
    return;
  }
  motor.linkDriver(&driver);

  // --- 1단계용 개루프 설정 ---
  motor.controller     = MotionControlType::velocity_openloop;
  motor.voltage_limit  = OPENLOOP_VOLTAGE;
  motor.velocity_limit = rpmToRps(RPM_CEILING);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.init();

  Serial.println();
  Serial.println(F("--- [STEP 1] Open-loop direction check ---"));
  Serial.println(F("Flywheel attached: using low speed and a gentle ramp."));
  Serial.print(F("Ramping FORWARD to "));
  Serial.print(DIR_TEST_RPM, 0);
  Serial.print(F(" RPM at "));
  Serial.print(DIR_RAMP_RPM_PER_S, 0);
  Serial.println(F(" RPM/s ..."));

  dir_cmd_rpm = 0;
  enterPhase(PH_DIR_FWD);
}

// initFOC 실행 및 1단계 결과와 교차 검증
void runFocInit() {
  // 코스트다운 단계에서 휠이 이미 멈춘 상태로 들어옵니다.
  // 개루프 정지 벡터(move(0))는 락드로터 과전류만 만들므로 주지 않습니다.
  motor.disable();
  delay(300);

  Serial.println();
  Serial.println(F("--- [STEP 2] initFOC alignment ---"));
  Serial.println(F("Watch the log below for 'sensor_direction' and 'PP check'."));

  motor.enable();
  motor.initFOC();

  if (motor.motor_status != FOCMotorStatus::motor_ready) {
    Serial.println(F("[FAIL] initFOC failed. Sensor alignment or pole pair (POLE_PAIRS) problem."));
    Serial.println(F("       Fix POLE_PAIRS using the 'estimated pp' value above, then re-upload."));
    enterPhase(PH_ABORT);
    return;
  }

  Serial.print(F("SimpleFOC detected sensor_direction = "));
  Serial.println(motor.sensor_direction == Direction::CW ? F("CW (+1)") : F("CCW (-1)"));
  Serial.print(F("zero_electric_angle = "));
  Serial.println(motor.zero_electric_angle, 4);

  // --- 3단계용 폐루프 속도제어 설정 ---
  motor.controller        = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_limit     = VOLTAGE_LIMIT;
  motor.velocity_limit    = rpmToRps(RPM_CEILING) + 10.0f;

  // 휠 관성이 크면 P가 높을수록 진동합니다. I도 낮춰야 와인드업으로 튀지 않습니다.
  motor.PID_velocity.P = 0.1f;   // 휠 없을 때 0.2 -> 장착 시 0.1
  motor.PID_velocity.I = 1.0f;   // 휠 없을 때 2.0 -> 장착 시 1.0
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 50.0f;  // 출력 전압 변화율 제한. 토크 스파이크 방지
  motor.LPF_velocity.Tf = 0.02f;           // 휠 진동 성분 필터링

  Serial.println();
  Serial.println(F("--- [STEP 3] Velocity ramp -> find max RPM ---"));
  Serial.print(F("Ramp rate "));
  Serial.print(RAMP_RATE_RPM_PER_S, 0);
  Serial.print(F(" RPM/s, step "));
  Serial.print(RAMP_STEP_RPM, 0);
  Serial.print(F(" RPM, settle "));
  Serial.print(SETTLE_MS / 1000.0f, 1);
  Serial.println(F(" s"));
  Serial.println(F("target_rpm\tactual_rpm\terr_rpm"));

  target_rpm      = 0;
  step_target_rpm = RAMP_STEP_RPM;
  best_actual_rpm = 0;
  saturate_count  = 0;
  last_step_ms    = millis();
  enterPhase(PH_RAMP);
}

void printDirectionResult() {
  Serial.println();
  Serial.println(F("--- [STEP 1] Result ---"));
  Serial.print(F("FORWARD cmd (+): AS5047 angle delta = "));
  Serial.print(dir_fwd_delta_deg, 1);
  Serial.println(F(" deg"));
  Serial.print(F("REVERSE cmd (-): AS5047 angle delta = "));
  Serial.print(dir_rev_delta_deg, 1);
  Serial.println(F(" deg"));

  if (fabs(dir_fwd_delta_deg) < DIR_MIN_DEG || fabs(dir_rev_delta_deg) < DIR_MIN_DEG) {
    Serial.println(F("[FAIL] Rotation too small. Motor is not spinning or sensor is not reading."));
    Serial.println(F("       Check: power supply, EN pin (GPIO12), AS5047 CS/SPI wiring, magnet gap."));
  } else if (dir_fwd_delta_deg > 0 && dir_rev_delta_deg < 0) {
    Serial.println(F("[PASS] Command sign matches sensor angle direction (no phase swap needed)."));
  } else if (dir_fwd_delta_deg < 0 && dir_rev_delta_deg > 0) {
    Serial.println(F("[NOTE] Command sign is INVERTED vs sensor angle direction."));
    Serial.println(F("       Not a fault. initFOC compensates with sensor_direction=CCW."));
    Serial.println(F("       To fix physically, swap any 2 of the 3 motor phase wires."));
  } else {
    Serial.println(F("[FAIL] Motor turned the SAME way for both forward and reverse commands."));
    Serial.println(F("       Likely open-loop sync loss. Raise OPENLOOP_VOLTAGE or lower DIR_TEST_RPM."));
  }
}

void printFinalResult() {
  // 휠이 돌고 있을 때 move(0)을 주면 급제동이라 큰 역토크와 과전류가 걸립니다.
  // 드라이버를 바로 끄고 자연 감속시킵니다.
  motor.disable();

  Serial.println();
  Serial.println(F("=========================================="));
  Serial.println(F(" FINAL RESULT"));
  Serial.println(F("=========================================="));
  Serial.print(F("Max measured RPM   : "));
  Serial.println(best_actual_rpm, 0);
  Serial.print(F("Target at saturation: "));
  Serial.print(target_rpm, 0);
  Serial.println(F(" RPM"));
  Serial.print(F("Voltage limit      : "));
  Serial.print(VOLTAGE_LIMIT, 1);
  Serial.print(F(" V (supply "));
  Serial.print(SUPPLY_VOLTAGE, 1);
  Serial.println(F(" V)"));
  Serial.println();
  Serial.println(F("Note: if actual RPM stops rising as target goes up, back-EMF has"));
  Serial.println(F("      consumed the full applied voltage. For higher RPM you need a"));
  Serial.println(F("      higher supply voltage or a higher-KV motor."));
  Serial.println(F("Driver disabled - flywheel is coasting down. Do NOT touch it."));
  Serial.println(F("Reset the board to run the test again."));
}

void loop() {
  // 종료 후에는 드라이버를 끈 채로 둡니다. move(0)을 계속 주면 휠에 제동이 걸립니다.
  if (phase == PH_DONE || phase == PH_ABORT) {
    return;
  }

  if (userAbortRequested()) {
    Serial.println();
    Serial.println(F("[ABORT] Stopped by user."));
    if (phase == PH_RAMP) printFinalResult();
    else {
      motor.disable();
      Serial.println(F("Driver disabled - flywheel is coasting down."));
    }
    phase = PH_ABORT;
    return;
  }

  unsigned long now = millis();

  switch (phase) {

    // ---------- 1단계: 개루프 정방향 ----------
    case PH_DIR_FWD: {
      float dt = (now - last_ramp_ms) / 1000.0f;
      last_ramp_ms = now;
      // 지령을 계단이 아니라 기울기로 올려야 관성이 붙은 휠이 동기를 유지합니다.
      dir_cmd_rpm += DIR_RAMP_RPM_PER_S * dt;
      if (dir_cmd_rpm > DIR_TEST_RPM) dir_cmd_rpm = DIR_TEST_RPM;
      motor.move(rpmToRps(dir_cmd_rpm));

      // 목표에 도달한 뒤부터 유지 시간을 세기 위해 램프 시간까지 포함해 판정
      unsigned long need = (unsigned long)(DIR_TEST_RPM / DIR_RAMP_RPM_PER_S * 1000.0f) + DIR_HOLD_MS;
      if (now - phase_start_ms >= need) {
        dir_fwd_delta_deg = readAngleDeg() - phase_start_angle;
        startCoast(PH_DIR_REV);
      }
      break;
    }

    // ---------- 1단계: 개루프 역방향 ----------
    case PH_DIR_REV: {
      float dt = (now - last_ramp_ms) / 1000.0f;
      last_ramp_ms = now;
      dir_cmd_rpm += DIR_RAMP_RPM_PER_S * dt;
      if (dir_cmd_rpm > DIR_TEST_RPM) dir_cmd_rpm = DIR_TEST_RPM;
      motor.move(-rpmToRps(dir_cmd_rpm));

      unsigned long need = (unsigned long)(DIR_TEST_RPM / DIR_RAMP_RPM_PER_S * 1000.0f) + DIR_HOLD_MS;
      if (now - phase_start_ms >= need) {
        dir_rev_delta_deg = readAngleDeg() - phase_start_angle;
        printDirectionResult();
        startCoast(PH_FOC_INIT);
      }
      break;
    }

    // ---------- 코스트다운: 드라이버 끄고 자연 감속 대기 ----------
    case PH_COAST: {
      float rpm = readSensorRpm();

      if (now - last_print_ms >= 500) {
        last_print_ms = now;
        Serial.print(F("  coasting... "));
        Serial.print(fabs(rpm), 0);
        Serial.println(F(" RPM"));
      }

      bool stopped   = fabs(rpm) < COAST_STOP_RPM;
      bool timed_out = (now - phase_start_ms) >= COAST_TIMEOUT_MS;

      if (stopped || timed_out) {
        if (timed_out && !stopped) {
          Serial.println(F("  coast timeout - proceeding anyway."));
        }
        motor.enable();
        dir_cmd_rpm = 0;
        // 다음 단계가 개루프 방향 테스트면 기준 각도를 여기서 다시 잡아야 정확합니다.
        if (coast_next == PH_DIR_REV) {
          Serial.print(F("Ramping REVERSE to "));
          Serial.print(DIR_TEST_RPM, 0);
          Serial.println(F(" RPM ..."));
        }
        enterPhase(coast_next);
      }
      break;
    }

    // ---------- 2단계: initFOC ----------
    case PH_FOC_INIT:
      runFocInit();
      break;

    // ---------- 3단계: 속도 램프 ----------
    case PH_RAMP: {
      motor.loopFOC();

      // 계단 목표(step_target_rpm)를 그대로 주지 않고, 기울기 제한을 걸어
      // 완만하게 따라가게 합니다. 휠 관성에 토크 스파이크가 걸리는 것을 막습니다.
      float dt = (now - last_ramp_ms) / 1000.0f;
      last_ramp_ms = now;
      bool at_step = false;
      if (target_rpm < step_target_rpm) {
        target_rpm += RAMP_RATE_RPM_PER_S * dt;
        if (target_rpm >= step_target_rpm) {
          target_rpm = step_target_rpm;
          last_step_ms = now;   // 목표 도달 시점부터 정착 시간 카운트 시작
        }
      } else {
        at_step = true;
      }

      motor.move(rpmToRps(target_rpm));

      float actual_rpm = rpsToRpm(motor.shaft_velocity);
      if (fabs(actual_rpm) > best_actual_rpm) best_actual_rpm = fabs(actual_rpm);

      if (now - last_print_ms >= 200) {
        last_print_ms = now;
        Serial.print(target_rpm, 0);
        Serial.print('\t');
        Serial.print(actual_rpm, 0);
        Serial.print('\t');
        Serial.println(target_rpm - actual_rpm, 0);
      }

      // 목표에 도달하고 정착 시간이 지난 뒤에만 포화 판정 및 다음 계단으로 진행
      if (at_step && (now - last_step_ms >= SETTLE_MS)) {
        last_step_ms = now;

        if (target_rpm > 0 && (target_rpm - actual_rpm) > TRACK_ERR_RPM) {
          saturate_count++;
          Serial.print(F("  ...tracking lost "));
          Serial.print(saturate_count);
          Serial.print('/');
          Serial.println(SATURATE_CONFIRM);
        } else {
          saturate_count = 0;
        }

        if (saturate_count >= SATURATE_CONFIRM) {
          Serial.println(F("Saturation detected - ramp finished."));
          printFinalResult();
          phase = PH_DONE;
          break;
        }

        if (target_rpm >= RPM_CEILING) {
          Serial.println(F("Safety ceiling (RPM_CEILING) reached - ramp finished."));
          Serial.println(F("If the motor still had headroom, raise RPM_CEILING and re-run."));
          printFinalResult();
          phase = PH_DONE;
          break;
        }

        step_target_rpm += RAMP_STEP_RPM;
        if (step_target_rpm > RPM_CEILING) step_target_rpm = RPM_CEILING;
      }
      break;
    }

    default:
      break;
  }
}
