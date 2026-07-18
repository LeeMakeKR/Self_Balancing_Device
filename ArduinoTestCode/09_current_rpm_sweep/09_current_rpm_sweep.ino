// 전류 제한별 최대 RPM 스윕 테스트 (MKS ESP32 FOC Mega 보드)
// 폐루프 속도제어(velocity) + foc_current 토크제어를 사용합니다.
//
// 동작 방식:
//   1) current_limit을 낮은 값(기본 0.1A)으로 설정
//   2) 목표 속도(target)를 0부터 서서히 올림
//   3) 실제 속도가 더 이상 목표를 따라 오르지 못하고 정체되면(=전류 한계로 포화)
//      그 지점의 최대 속도를 기록
//   4) current_limit을 한 단계(기본 0.1A) 올리고 다시 0부터 반복
//   5) 안전 상한(MAX_CURRENT_A) 도달 또는 속도 상한 도달 시 종료, 결과 테이블 출력
//
// AS5047P(SPI) 센서 + 보드 내장 전류센서(INA240 x2) 필요 (06/07/08과 동일 하드웨어).
// 06_stepcommutation 자동진단 결과 반영: 극쌍수(PP)=5, 회전방향 정방향 일치.

#include <SimpleFOC.h>

// ===== 핀 정의 (MKS ESP32 FOC Mega 보드 실측) =====
#define PIN_A 32
#define PIN_B 33
#define PIN_C 25
#define PIN_EN 12

#define PIN_CS0_A 39  // A상 전류센서 (SENSOR_VN)
#define PIN_CS0_B 36  // B상 전류센서 (SENSOR_VP)

#define PIN_SPI_SCLK 18
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 23
#define PIN_SPI_CS   5

// ===== 스윕 테스트 파라미터 =====
const float START_CURRENT_A = 0.1f;   // 시작 전류 제한 [A]
const float CURRENT_STEP_A  = 0.1f;   // 전류 제한 증가 단위 [A]
const float MAX_CURRENT_A   = 1.5f;   // 안전 상한 [A] (필요 시 조정)

const float TARGET_STEP_RPS   = 5.0f;   // 램프 시 목표 속도 증가량 [rad/s]
const unsigned long RAMP_INTERVAL_MS = 300; // 목표 증가 간격(정착 대기) [ms]
const float VELOCITY_CEILING_RPS = 1047.2f; // 목표 속도 안전 상한 [rad/s] (10000rpm = rpm*2*PI/60)

const float STALL_GAIN_THRESHOLD_RPS = 3.0f; // 이보다 실제속도 증가가 작으면 "정체"로 판단
const int   STALL_CONFIRM_COUNT = 3;         // 연속 정체 확인 횟수 (노이즈 방지)

// ===== 객체 =====
BLDCMotor motor = BLDCMotor(5);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_A, PIN_B, PIN_C, PIN_EN);
InlineCurrentSense current_sense = InlineCurrentSense(500.0f, PIN_CS0_A, PIN_CS0_B);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, PIN_SPI_CS);

Commander command = Commander(Serial);
void onMotor(char* cmd) { command.motor(&motor, cmd); }

bool userAbortRequested() {
  while (Serial.available()) {
    if ((char)Serial.read() == 'x') return true;
  }
  return false;
}

float rpsToRpm(float rps) { return rps * 60.0f / (2.0f * PI); }

// ===== 결과 저장 =====
#define MAX_LEVELS 32
float result_current[MAX_LEVELS];
float result_max_rps[MAX_LEVELS];
int result_count = 0;

// ===== 상태 머신 =====
enum SweepState { SWEEP_LEVEL_START, SWEEP_RAMPING, SWEEP_DONE, SWEEP_ABORTED };
SweepState state = SWEEP_LEVEL_START;

float current_level = START_CURRENT_A;
float ramp_target = 0;
float best_velocity = 0;
float prev_velocity = 0;
int stall_count = 0;

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // 센서 초기화
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);
  sensor.init();
  motor.linkSensor(&sensor);

  // 드라이버 초기화
  driver.voltage_power_supply = 12.0;
  driver.voltage_limit = 11.5; // 공급전압(9V) 근처까지 상향 - RPM이 전류가 아니라 전압(역기전력) 한계에 막히지 않도록
  if (!driver.init()) {
    Serial.println("드라이버 초기화 실패!");
    return;
  }
  motor.linkDriver(&driver);

  // 전류센서 초기화 (드라이버 disable 상태에서 오프셋 캘리브레이션)
  current_sense.linkDriver(&driver);
  if (!current_sense.init()) {
    Serial.println("전류센서 초기화 실패!");
    return;
  }
  current_sense.skip_align = true;
  motor.linkCurrentSense(&current_sense);

  // 폐루프 속도 제어 + foc_current 토크 제어
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::foc_current;

  motor.voltage_limit = 11.5;
  motor.velocity_limit = VELOCITY_CEILING_RPS + 10; // 램프 상한보다 여유있게
  motor.current_limit = START_CURRENT_A;

  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 2.0f;
  motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = 0.01f;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.init();
  motor.initFOC();
  motor.target = 0;

  command.add('M', onMotor, "motor");

  Serial.println(F("=== 전류 제한별 최대 RPM 스윕 테스트 시작 ==="));
  Serial.print(F("시작 전류: ")); Serial.print(START_CURRENT_A, 2);
  Serial.print(F("A, 단위: ")); Serial.print(CURRENT_STEP_A, 2);
  Serial.print(F("A, 상한: ")); Serial.print(MAX_CURRENT_A, 2);
  Serial.println(F("A"));
  Serial.println(F("(중단하려면 시리얼 모니터에 x 입력)"));
  _delay(1000);
}

void runSweep() {
  if (userAbortRequested()) {
    motor.target = 0;
    motor.disable();
    Serial.println(F("[중단] 사용자 요청으로 테스트를 멈췄습니다."));
    state = SWEEP_ABORTED;
    return;
  }

  switch (state) {
    case SWEEP_LEVEL_START: {
      if (current_level > MAX_CURRENT_A + 1e-3f) {
        state = SWEEP_DONE;
        return;
      }
      motor.updateCurrentLimit(current_level); // 필드 직접 대입 대신 반드시 이 함수로 설정해야
                                                // PID_velocity.limit(속도루프 출력 제한)에도 반영됨
      ramp_target = 0;
      best_velocity = 0;
      prev_velocity = 0;
      stall_count = 0;
      motor.target = 0;

      Serial.print(F("\n[레벨 시작] current_limit = "));
      Serial.print(current_level, 2);
      Serial.println(F("A"));

      delay(500); // 이전 레벨에서 감속 정착 대기
      state = SWEEP_RAMPING;
      return;
    }

    case SWEEP_RAMPING: {
      // FOC 루프는 별도로 빠르게 돌려야 하므로, 짧은 주기로 여러 번 loopFOC/move 호출
      unsigned long t0 = millis();
      while (millis() - t0 < RAMP_INTERVAL_MS) {
        motor.loopFOC();
        motor.move();
      }

      motor.loopFOC();
      float actual_velocity = fabs(motor.shaft_velocity);

      Serial.print("  target="); Serial.print(ramp_target, 2);
      Serial.print("rad/s  actual="); Serial.print(actual_velocity, 2);
      Serial.print("rad/s ("); Serial.print(rpsToRpm(actual_velocity), 0);
      Serial.println("rpm)");

      if (actual_velocity > best_velocity) best_velocity = actual_velocity;

      float gain = actual_velocity - prev_velocity;
      prev_velocity = actual_velocity;

      bool target_reached_ceiling = ramp_target >= VELOCITY_CEILING_RPS;
      bool stalled = false;

      if (ramp_target > 1.0f) { // 초반 가속 구간은 정체 판정에서 제외
        if (gain < STALL_GAIN_THRESHOLD_RPS) {
          stall_count++;
          if (stall_count >= STALL_CONFIRM_COUNT) stalled = true;
        } else {
          stall_count = 0;
        }
      }

      if (stalled || target_reached_ceiling) {
        // 이번 레벨 결과 기록
        if (result_count < MAX_LEVELS) {
          result_current[result_count] = current_level;
          result_max_rps[result_count] = best_velocity;
          result_count++;
        }

        Serial.print(F("  -> 결과: current_limit="));
        Serial.print(current_level, 2);
        Serial.print(F("A 에서 최대 "));
        Serial.print(best_velocity, 2);
        Serial.print(F("rad/s ("));
        Serial.print(rpsToRpm(best_velocity), 0);
        Serial.println(F("rpm)"));

        if (target_reached_ceiling) {
          Serial.println(F("  (속도 안전상한 도달 - 전류 한계가 아니라 설정 상한 도달. 스윕 조기 종료.)"));
          motor.target = 0;
          state = SWEEP_DONE;
          return;
        }

        // 다음 전류 레벨로
        current_level += CURRENT_STEP_A;
        state = SWEEP_LEVEL_START;
        return;
      }

      // 아직 정체 아님 -> 목표 속도 한 단계 더 증가
      ramp_target += TARGET_STEP_RPS;
      motor.target = ramp_target;
      return;
    }

    default:
      return;
  }
}

void printSummary() {
  Serial.println(F("\n=== 전류 제한별 최대 RPM 결과 요약 ==="));
  Serial.println(F("current_limit[A]\tmax_velocity[rad/s]\tmax_rpm"));
  for (int i = 0; i < result_count; i++) {
    Serial.print(result_current[i], 2);
    Serial.print("\t\t");
    Serial.print(result_max_rps[i], 2);
    Serial.print("\t\t");
    Serial.println(rpsToRpm(result_max_rps[i]), 0);
  }
}

void loop() {
  if (state == SWEEP_LEVEL_START || state == SWEEP_RAMPING) {
    runSweep();
  } else if (state == SWEEP_DONE) {
    static bool printed = false;
    if (!printed) {
      motor.target = 0;
      motor.disable();
      printSummary();
      printed = true;
    }
  } else {
    // SWEEP_ABORTED: 정지 상태 유지
  }

  command.run();
}
