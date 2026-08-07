// 가속 램프율 자동 튜닝 (MKS ESP32 FOC Mega + AS5047P)
//
// 목적: 목표 RPM까지 최단 시간에 올리는 가속률을 사람 개입 없이 찾아냅니다.
//
// 동작:
//   가속률을 낮은 값부터 RAMP_FACTOR배씩 올려가며 매 시행마다
//     1) 0 -> TARGET_RPM 가속, 95% 도달 시간(t95) 측정
//     2) 가속 중 지령-실제 최대 뒤처짐(max lag) 기록
//     3) 드라이버를 끄고 자유 코스팅으로 정지 (LED 적색 점등)
//   전부 끝나면 결과 표를 출력하고 최적 가속률을 추천합니다.
//
// 판정 기준:
//   가속률을 올리면 t95가 줄다가 어느 지점부터 더 안 줄어듭니다.
//   그 지점부터는 모터 토크가 한계라 지령만 앞서갈 뿐입니다.
//   따라서 "최소 t95의 105% 이내를 달성하는 가장 낮은 가속률"을 최적으로 봅니다.
//   지령이 실제보다 LAG_LIMIT_RPM 넘게 앞서간 시행은 탈락시킵니다.
//
// 제동 방식:
//   이 전원 구성은 회생 전력을 흡수하지 못하므로 능동 감속(제어 감속)을 쓰지 않습니다.
//   드라이버를 완전히 끄고 마찰로만 감속시킵니다. 전류가 전원으로 되돌아가지 않습니다.
//   대신 감속이 느립니다. TARGET_RPM을 올리면 코스팅 시간이 그만큼 길어집니다.
//
// 상태 LED (보드 내장 WS2812, GPIO2):
//   녹색 = 가속 중,  적색 = 제동(코스팅) 중,  소등 = 대기/종료
//
// 부수 확인: actual 열이 cmd를 따라오면 모터 정상, angle 열이 계속 변하면 센서 정상.
//
// 시리얼 115200. 'x' 입력 시 즉시 중단하고 그때까지의 결과를 출력.

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

#define PIN_LED 2   // 보드 내장 WS2812 상태 LED

// ===== 모터 파라미터 =====
#define POLE_PAIRS 10   // initFOC의 "PP check" 경고가 뜨면 로그의 est. pp 값으로 교체

// ===== 전원 / 안전 =====
const float SUPPLY_VOLTAGE = 12.0f;
const float VOLTAGE_LIMIT  = 11.0f;
const float RPM_CEILING    = 6000.0f;

// ===== 스윕 설정 =====
// 주의: 회생 제동을 못 쓰므로 감속은 마찰뿐입니다. TARGET_RPM에 비례해
// 시행당 코스팅 시간이 길어집니다. 3000 RPM이면 시행당 수 분이 걸릴 수 있어
// 전체 테스트가 길어집니다(진행 상황은 1초마다 출력됨).
const float TARGET_RPM  = 3000.0f;
const float RAMP_START  = 200.0f;
const float RAMP_FACTOR = 1.6f;
const float RAMP_MAX    = 12000.0f;
#define MAX_TRIALS 10

// ===== 판정 파라미터 =====
const float         REACH_FRACTION   = 0.95f;  // 목표의 95% 도달을 성공으로
const float         LAG_LIMIT_RPM    = 300.0f; // 지령이 실제보다 이만큼 앞서면 탈락
const unsigned long TRIAL_TIMEOUT_MS = 20000;
const float         KNEE_TOLERANCE   = 1.05f;  // 최소 t95의 몇 배까지 "동급"으로 볼지

// ===== 코스팅 / 정착 =====
// 정지 판정은 고정 시간창(COAST_SAMPLE_MS) 동안의 각도 변화로 계산합니다.
// sensor.getVelocity()를 매 루프 읽으면 양자화 노이즈로 순간 0이 튀어나와
// 아직 돌고 있는데도 정지로 오판합니다.
// 정지 판정은 "거의 멈춤"이 아니라 "완전히 멈춤"이어야 합니다.
// 30 RPM(약 3.1 rad/s)은 휠이 눈에 띄게 돌고 있는 속도라, 그 상태에서 다음 시행을
// 시작하면 초기 속도가 0이 아닌 채로 t95를 재게 됩니다.
//
// 0.5 RPM은 200ms 창에서 각도 변화 약 0.6도에 해당합니다.
// AS5047의 14bit 분해능이 0.022도이므로 노이즈보다 충분히 큽니다.
const float         STOP_RPM         = 0.5f;    // 이 아래면 정지 후보
const int           STOP_CONFIRM     = 3;       // 연속 몇 번 확인해야 정지로 확정
const unsigned long COAST_SAMPLE_MS  = 200;     // 속도 계산 시간창
const unsigned long COAST_TIMEOUT_MS = 120000;  // 자유 감속은 오래 걸립니다
const unsigned long SETTLE_MS        = 1000;

// ===== LED 밝기 =====
const uint8_t LED_LEVEL = 40;   // 0-255. 눈부심과 소비전류를 줄이려 낮게

// ===== 객체 =====
BLDCMotor         motor  = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM    driver = BLDCDriver3PWM(PIN_A, PIN_B, PIN_C, PIN_EN);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, PIN_SPI_CS);

// ===== 상태 머신 =====
// 주의: enum은 첫 함수 정의보다 위에 있어야 합니다.
// Arduino IDE가 자동 생성하는 프로토타입이 첫 함수 앞에 삽입되기 때문입니다.
enum TuneState {
  ST_ACCEL,    // 0 -> TARGET_RPM 가속하며 측정
  ST_COAST,    // 드라이버 off, 자유 감속 (LED 적색)
  ST_SETTLE,   // 정지 확인 후 짧은 대기
  ST_REPORT,
  ST_DONE
};
TuneState state = ST_ACCEL;

// ===== 단위 변환 =====
float rpmToRps(float rpm) { return rpm * 2.0f * PI / 60.0f; }
float rpsToRpm(float rps) { return rps * 60.0f / (2.0f * PI); }

// ===== LED 제어 =====
// ESP32 Arduino 코어 3.x 내장 함수. 별도 라이브러리가 필요 없습니다.
void ledOff()     { neopixelWrite(PIN_LED, 0, 0, 0); }
void ledBrake()   { neopixelWrite(PIN_LED, LED_LEVEL, 0, 0); }  // 적색: 멈춰야 하는 중
void ledStopped() { neopixelWrite(PIN_LED, 0, LED_LEVEL, 0); }  // 녹색: 정지 확인됨

// 부팅 시 LED 배선/핀 확인용. 색이 안 보이면 PIN_LED가 틀린 것입니다.
void ledSelfTest() {
  neopixelWrite(PIN_LED, LED_LEVEL, 0, 0); delay(400);   // R
  neopixelWrite(PIN_LED, 0, LED_LEVEL, 0); delay(400);   // G
  neopixelWrite(PIN_LED, 0, 0, LED_LEVEL); delay(400);   // B
  ledOff();
}

// ===== 시행 상태 =====
int           trial_idx   = 0;
float         ramp_rate   = RAMP_START;
float         cmd_rpm     = 0;
bool          reached     = false;
float         max_lag     = 0;
unsigned long state_start = 0;
unsigned long last_ms     = 0;
unsigned long last_print  = 0;

// 코스팅 정지 판정용
float         coast_last_angle  = 0;
unsigned long coast_last_sample = 0;
int           stop_count        = 0;

// ===== 결과 =====
float         res_ramp[MAX_TRIALS];
unsigned long res_t95[MAX_TRIALS];    // 0 = 도달 실패
float         res_lag[MAX_TRIALS];
unsigned long res_coast[MAX_TRIALS];  // 시행별 코스팅 소요 시간 [ms]
int           res_count = 0;

// ===== 최종 보고서용 누적 통계 =====
float peak_rpm       = 0;   // 전 시행 통틀어 관측된 최고 실측 RPM
float boot_angle_rad = 0;   // 부팅 시점 누적 각도 (총 회전량 계산용)

bool userAbortRequested() {
  while (Serial.available()) {
    if ((char)Serial.read() == 'x') return true;
  }
  return false;
}

// 누적 각도(회전수 포함)를 rad 단위로 반환
float readAngleRad() {
  return sensor.getAngle();
}

void enterState(TuneState next) {
  state = next;
  state_start = millis();
  last_ms = state_start;
}

void startTrial() {
  motor.PID_velocity.reset();   // 이전 시행의 적분항이 남으면 시작하자마자 튑니다
  motor.enable();
  ledOff();                     // 가속 구간은 소등. LED는 제동/정지 상태만 표시합니다.

  cmd_rpm = 0;
  reached = false;
  max_lag = 0;

  Serial.println();
  Serial.print(F(">>> trial "));
  Serial.print(trial_idx + 1);
  Serial.print(F("  ramp = "));
  Serial.print(ramp_rate, 0);
  Serial.println(F(" RPM/s"));
  Serial.println(F("ms\tcmd\tactual\tangle"));

  enterState(ST_ACCEL);
}

void recordTrial(unsigned long t95) {
  if (res_count < MAX_TRIALS) {
    res_ramp[res_count]  = ramp_rate;
    res_t95[res_count]   = t95;
    res_lag[res_count]   = max_lag;
    res_coast[res_count] = 0;   // 코스팅이 끝날 때 채워집니다
    res_count++;
  }

  Serial.print(F("    result: "));
  if (t95 == 0) Serial.print(F("FAIL (timeout)"));
  else { Serial.print(t95); Serial.print(F(" ms")); }
  Serial.print(F("   max lag = "));
  Serial.print(max_lag, 0);
  Serial.print(F(" RPM"));
  if (max_lag > LAG_LIMIT_RPM) Serial.print(F("  [rejected: cmd outran motor]"));
  Serial.println();
}

// 드라이버를 끄고 자유 감속 구간으로 진입. 회생 전력이 전원으로 돌아가지 않습니다.
void startCoast() {
  motor.move(0);
  motor.disable();
  ledBrake();   // 적색: 멈춰야 하는 중

  sensor.update();
  coast_last_angle  = readAngleRad();
  coast_last_sample = millis();
  stop_count        = 0;

  Serial.println(F("    [BRAKE] driver off, coasting down (LED RED)..."));
  enterState(ST_COAST);
}

void printSeparator() {
  Serial.println(F("=============================================================="));
}

// 모든 측정 결과를 한 곳에 모아 출력하는 최종 보고서
void printReport() {
  motor.disable();
  ledOff();

  // --- 집계 ---
  unsigned long best_t    = 0;   // 최소 t95
  float         best_ramp = 0;   // 그 t95를 낸 램프율
  int           ok_count  = 0;
  for (int i = 0; i < res_count; i++) {
    if (res_t95[i] > 0 && res_lag[i] <= LAG_LIMIT_RPM) {
      ok_count++;
      if (best_t == 0 || res_t95[i] < best_t) { best_t = res_t95[i]; best_ramp = res_ramp[i]; }
    }
  }

  // 무릎점: 최소 t95의 KNEE_TOLERANCE 배 이내를 달성하는 가장 낮은 가속률.
  // 배열이 낮은 값부터 채워지므로 첫 매치가 곧 무릎점입니다.
  float knee = 0;
  for (int i = 0; i < res_count; i++) {
    if (res_t95[i] > 0 && res_lag[i] <= LAG_LIMIT_RPM &&
        res_t95[i] <= (unsigned long)(best_t * KNEE_TOLERANCE)) { knee = res_ramp[i]; break; }
  }

  sensor.update();
  float total_rev = fabs(readAngleRad() - boot_angle_rad) / _2PI;

  Serial.println();
  Serial.println();
  printSeparator();
  Serial.println(F("                    FINAL REPORT"));
  printSeparator();

  // --- [1] 설정 ---
  Serial.println(F("[1] CONFIGURATION"));
  Serial.print(F("  pole pairs       : ")); Serial.println(POLE_PAIRS);
  Serial.print(F("  supply / limit   : "));
  Serial.print(SUPPLY_VOLTAGE, 1); Serial.print(F(" / "));
  Serial.print(VOLTAGE_LIMIT, 1);  Serial.println(F(" V"));
  Serial.print(F("  target speed     : ")); Serial.print(TARGET_RPM, 0); Serial.println(F(" RPM"));
  Serial.print(F("  sweep range      : "));
  Serial.print(RAMP_START, 0); Serial.print(F(" -> "));
  Serial.print(RAMP_MAX, 0);   Serial.print(F(" RPM/s  (x"));
  Serial.print(RAMP_FACTOR, 1); Serial.println(F(")"));
  Serial.print(F("  sensor direction : "));
  Serial.println(motor.sensor_direction == Direction::CW ? F("CW (+1)") : F("CCW (-1)"));
  Serial.print(F("  zero elec. angle : ")); Serial.println(motor.zero_electric_angle, 4);
  Serial.print(F("  PID P / I        : "));
  Serial.print(motor.PID_velocity.P, 3); Serial.print(F(" / "));
  Serial.println(motor.PID_velocity.I, 3);
  Serial.println();

  // --- [2] 하드웨어 건전성 ---
  Serial.println(F("[2] HEALTH CHECK"));
  Serial.print(F("  motor rotated    : "));
  if (peak_rpm > 50.0f) { Serial.print(F("YES  (peak ")); Serial.print(peak_rpm, 0); Serial.println(F(" RPM)")); }
  else                  { Serial.println(F("NO - motor never spun up. Check POLE_PAIRS / wiring.")); }
  Serial.print(F("  sensor readings  : "));
  if (total_rev > 1.0f) { Serial.print(F("OK   (total ")); Serial.print(total_rev, 1); Serial.println(F(" rev)")); }
  else                  { Serial.println(F("NO CHANGE - check AS5047 SPI wiring / magnet gap.")); }
  Serial.print(F("  trials run       : "));
  Serial.print(res_count); Serial.print(F("  ("));
  Serial.print(ok_count);  Serial.print(F(" ok / "));
  Serial.print(res_count - ok_count); Serial.println(F(" rejected)"));
  Serial.println();

  // --- [3] 시행 표 ---
  Serial.println(F("[3] TRIAL TABLE"));
  Serial.println(F("  ramp[RPM/s]\tt95[ms]\tmaxlag\tcoast[s]\tverdict"));
  for (int i = 0; i < res_count; i++) {
    bool ok = (res_t95[i] > 0) && (res_lag[i] <= LAG_LIMIT_RPM);
    Serial.print(F("  "));
    Serial.print(res_ramp[i], 0);  Serial.print('\t');
    if (res_t95[i] == 0) Serial.print(F("-"));
    else                 Serial.print(res_t95[i]);
    Serial.print('\t');
    Serial.print(res_lag[i], 0);   Serial.print('\t');
    Serial.print(res_coast[i] / 1000.0f, 1); Serial.print('\t');
    if (!ok)                       Serial.println(F("rejected"));
    else if (res_ramp[i] == knee)  Serial.println(F("ok  <== BEST"));
    else                           Serial.println(F("ok"));
  }
  Serial.println();

  // --- [4] 분석 ---
  Serial.println(F("[4] ANALYSIS"));
  if (best_t == 0) {
    Serial.println(F("  No valid trial - the motor never reached the target."));
    Serial.println(F("  Lower TARGET_RPM, or raise VOLTAGE_LIMIT / supply voltage."));
    printSeparator();
    return;
  }
  Serial.print(F("  fastest t95      : "));
  Serial.print(best_t); Serial.print(F(" ms  (at "));
  Serial.print(best_ramp, 0); Serial.println(F(" RPM/s)"));
  Serial.print(F("  recommended ramp : "));
  Serial.print(knee, 0); Serial.println(F(" RPM/s"));
  Serial.println(F("  Rates above this do not arrive sooner - the motor is torque-"));
  Serial.println(F("  limited past that point, so the command just runs ahead."));
  Serial.println();

  // --- [5] 적용 ---
  Serial.println(F("[5] APPLY - paste into your control sketch"));
  Serial.print(F("  const float RAMP_RATE_RPM_PER_S = "));
  Serial.print(knee, 0);
  Serial.println(F(";"));
  Serial.println();

  printSeparator();
  Serial.println(F("Driver disabled. Reset the board to run again."));
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  SimpleFOCDebug::enable(&Serial);

  Serial.println(F("=========================================="));
  Serial.println(F(" Automatic Accel Ramp Tuning"));
  Serial.println(F(" Braking is passive coast-down (no regen)"));
  Serial.println(F(" LED: RED = braking, GREEN = stop confirmed"));
  Serial.println(F(" Send 'x' at any time to abort"));
  Serial.println(F("=========================================="));

  // LED 배선 확인: R -> G -> B 순서로 점등합니다.
  // 아무 색도 안 보이면 PIN_LED 번호가 틀렸거나 LED가 WS2812가 아닙니다.
  Serial.println(F("LED self-test: red, green, blue..."));
  ledSelfTest();

  // --- 센서 ---
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);
  sensor.clock_speed = 10000000;   // 기본 1MHz는 고RPM에서 병목
  sensor.init();
  motor.linkSensor(&sensor);

  // --- 드라이버 ---
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = VOLTAGE_LIMIT;
  if (!driver.init()) {
    Serial.println(F("[FAIL] driver init"));
    while (1) delay(100);
  }
  motor.linkDriver(&driver);

  // --- 폐루프 속도제어 ---
  motor.controller        = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation    = FOCModulationType::SpaceVectorPWM;
  motor.voltage_limit     = VOLTAGE_LIMIT;
  motor.velocity_limit    = rpmToRps(RPM_CEILING);

  motor.PID_velocity.P           = 0.1f;
  motor.PID_velocity.I           = 1.0f;
  motor.PID_velocity.D           = 0.0f;
  motor.PID_velocity.output_ramp = 1000.0f;  // 램프율 자체를 재는 테스트이므로
                                             // 출력단 제한은 풀어둡니다
  motor.LPF_velocity.Tf          = 0.02f;

  motor.init();
  motor.initFOC();

  if (motor.motor_status != FOCMotorStatus::motor_ready) {
    Serial.println(F("[FAIL] initFOC. Check POLE_PAIRS and sensor wiring."));
    while (1) delay(100);
  }

  sensor.update();
  boot_angle_rad = readAngleRad();   // 총 회전량 계산 기준점
  Serial.print(F("sensor angle at boot = "));
  Serial.print(boot_angle_rad * 180.0f / PI, 1);
  Serial.println(F(" deg"));

  Serial.print(F("sweep: "));
  Serial.print(RAMP_START, 0);
  Serial.print(F(" RPM/s x"));
  Serial.print(RAMP_FACTOR, 1);
  Serial.print(F(" up to "));
  Serial.print(RAMP_MAX, 0);
  Serial.println(F(" RPM/s"));

  startTrial();
}

void loop() {
  if (state == ST_DONE) return;

  if (userAbortRequested()) {
    Serial.println();
    Serial.println(F("[ABORT] stopped by user."));
    motor.disable();
    ledBrake();   // 휠이 아직 돌고 있으므로 제동 표시 유지
    printReport();
    state = ST_DONE;
    return;
  }

  motor.loopFOC();

  unsigned long now = millis();
  float dt = (now - last_ms) / 1000.0f;
  last_ms = now;

  switch (state) {

    // ---------- 가속: 램프율대로 지령을 올리며 도달 시간 측정 ----------
    case ST_ACCEL: {
      cmd_rpm += ramp_rate * dt;
      if (cmd_rpm > TARGET_RPM) cmd_rpm = TARGET_RPM;
      motor.move(rpmToRps(cmd_rpm));

      float actual_rpm = rpsToRpm(motor.shaft_velocity);
      float lag = cmd_rpm - actual_rpm;
      if (lag > max_lag) max_lag = lag;
      if (actual_rpm > peak_rpm) peak_rpm = actual_rpm;   // 최종 보고서용

      if (now - last_print >= 200) {
        last_print = now;
        Serial.print(now - state_start); Serial.print('\t');
        Serial.print(cmd_rpm, 0);        Serial.print('\t');
        Serial.print(actual_rpm, 0);     Serial.print('\t');
        Serial.println(sensor.getAngle() * 180.0f / PI, 0);
      }

      if (!reached && actual_rpm >= TARGET_RPM * REACH_FRACTION) {
        reached = true;
        recordTrial(now - state_start);
        startCoast();
        break;
      }

      if (now - state_start >= TRIAL_TIMEOUT_MS) {
        recordTrial(0);   // 0 = 실패
        startCoast();
      }
      break;
    }

    // ---------- 제동: 드라이버 off, 마찰로만 감속 (LED 적색) ----------
    case ST_COAST: {
      // 고정 시간창 동안의 각도 변화로 속도를 계산합니다.
      // 매 루프 getVelocity()를 읽으면 양자화 노이즈로 순간 0이 나와 오판합니다.
      if (now - coast_last_sample >= COAST_SAMPLE_MS) {
        float angle = readAngleRad();
        float dt_s  = (now - coast_last_sample) / 1000.0f;
        float rpm   = fabs(rpsToRpm((angle - coast_last_angle) / dt_s));

        coast_last_angle  = angle;
        coast_last_sample = now;

        if (rpm < STOP_RPM) stop_count++;
        else                stop_count = 0;

        if (now - last_print >= 1000) {
          last_print = now;
          Serial.print(F("    coasting "));
          Serial.print((now - state_start) / 1000);
          Serial.print(F("s ... "));
          Serial.print(rpm, 2);
          Serial.print(F(" RPM  (stop "));
          Serial.print(stop_count);
          Serial.print('/');
          Serial.print(STOP_CONFIRM);
          Serial.println(')');
        }
      }

      // 시간 기준을 걸지 않고 센서 판정만으로 결정합니다.
      // 0.5 RPM 임계 + 3창 연속(600ms) 확인이면 감속 직후 노이즈로 조기 종료할 수 없습니다.
      bool stopped   = (stop_count >= STOP_CONFIRM);
      bool timed_out = (now - state_start) >= COAST_TIMEOUT_MS;

      if (stopped || timed_out) {
        if (timed_out && !stopped) {
          Serial.println(F("    coast timeout - proceeding anyway."));
        } else {
          Serial.println(F("    [STOPPED] confirmed at rest (LED GREEN)"));
        }
        // 방금 끝난 시행의 코스팅 시간을 최종 보고서용으로 기록
        if (res_count > 0) res_coast[res_count - 1] = now - state_start;

        ledStopped();   // 녹색: 정지 확인됨
        enterState(ST_SETTLE);
      }
      break;
    }

    // ---------- 정지 확인 후 대기 (LED 녹색 유지) ----------
    case ST_SETTLE: {
      if (now - state_start >= SETTLE_MS) {
        trial_idx++;
        ramp_rate *= RAMP_FACTOR;

        if (trial_idx >= MAX_TRIALS || ramp_rate > RAMP_MAX) enterState(ST_REPORT);
        else                                                 startTrial();
      }
      break;
    }

    case ST_REPORT:
      printReport();
      state = ST_DONE;
      break;

    default:
      break;
  }
}
