// 자동 하드웨어 진단 테스트 (MKS ESP32 FOC Mega 보드)
// AS5047P(SPI 각도센서) + 보드 내장 전류센서(INA240 x2)를 이용해
// 3단계로 나누어 자동으로 하드웨어 상태를 점검합니다.
//
//   Stage 1: 6-step 벡터 각각에 대해 실제로 상별 전류가 흐르는지 확인 (통전 확인)
//   Stage 2: 전류가 흐를 때 축이 실제로 조금이라도 움직이는지 확인 (구동력 확인)
//   Stage 3: 여러 바퀴 연속 회전시키며 누적 각도 변화 방향으로 상 순서/회전방향 판별
//
// 각 단계는 이전 단계를 통과해야 다음 단계로 진행되며, 실패 시 어느 스텝에서
// 어떤 값으로 실패했는지 로그를 남기고 드라이버를 즉시 정지합니다.
//
// 이 테스트는 저전압(기본 3V)으로 짧게 여러 번 통전하는 방식이지만,
// 배선이 잘못된 상태(쇼트 등)일 수 있으므로 최초 실행 시 반드시 옆에서 지켜보세요.

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

// ===== 테스트 파라미터 =====
// 정지된 상태로 벡터를 유지하는 것은 역기전력이 없는 사실상 locked-rotor 조건이라
// 낮은 전압에서도 전류가 크게 흐를 수 있습니다. 통전 확인 목적이면 1V 내외로 충분합니다.
const float TEST_VOLTAGE = 1.0f;             // 테스트 인가 전압 [V]
const unsigned long SETTLE_MS = 150;         // 벡터 인가 후 전류 안정화 대기 [ms]
const unsigned long STAGE1_HOLD_MS = 400;    // Stage1 스텝당 총 유지 시간 [ms]
const unsigned long STAGE2_HOLD_MS = 500;    // Stage2 스텝당 총 유지 시간 [ms] (움직임 관찰용, 조금 더 김)
const unsigned long STAGE3_STEP_MS = 300;    // Stage3 스텝당 유지 시간 [ms]
const int STAGE3_CYCLES = 3;                 // Stage3 연속 회전 바퀴 수(6-step 기준)

const float CURRENT_THRESHOLD_A = 0.05f;     // 통전 판정 임계값 [A]
const float CURRENT_FAULT_A = 1.0f;          // 과전류 안전 차단 임계값 [A]
const float ANGLE_MOVE_THRESHOLD_RAD = 0.02f; // 움직임 판정 임계값 [rad] (약 1.1도)

// ===== 객체 =====
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_A, PIN_B, PIN_C, PIN_EN);
InlineCurrentSense current_sense = InlineCurrentSense(500.0f, PIN_CS0_A, PIN_CS0_B); // INA240A2(50V/V) x 10mΩ 션트 = 500mV/A
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, PIN_SPI_CS);

// ===== 상태 머신 =====
enum TestStage { STAGE1_CONTINUITY, STAGE2_MOVEMENT, STAGE3_DIRECTION, TEST_PASSED, TEST_FAILED };
TestStage stage = STAGE1_CONTINUITY;

// 전기각(angle_el)에 대응하는 고정 전압 벡터를 3상에 인가 (역파크+역클라크 변환)
void setVector(float Uq, float angle_el) {
  float Ualpha = -Uq * sin(angle_el);
  float Ubeta  =  Uq * cos(angle_el);

  float Ua = Ualpha + driver.voltage_limit / 2;
  float Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + driver.voltage_limit / 2;
  float Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + driver.voltage_limit / 2;

  driver.setPwm(Ua, Ub, Uc);
}

// 사용자가 시리얼에 'x'를 입력하면 즉시 중단 (비상정지)
bool userAbortRequested() {
  while (Serial.available()) {
    if ((char)Serial.read() == 'x') return true;
  }
  return false;
}

void haltOnFault(const char* msg) {
  driver.setPwm(0, 0, 0);
  driver.disable();
  Serial.print(F("[FAULT] "));
  Serial.println(msg);
  stage = TEST_FAILED;
}

// 과전류 감지 시 즉시 정지, true 반환. 실측값은 항상 먼저 출력해 원인 파악에 활용.
bool checkOvercurrent(PhaseCurrent_s c) {
  float ic = -(c.a + c.b);
  Serial.print("    (실측: Ia="); Serial.print(c.a, 3);
  Serial.print("A Ib="); Serial.print(c.b, 3);
  Serial.print("A Ic="); Serial.print(ic, 3);
  Serial.println("A)");

  if (fabs(c.a) > CURRENT_FAULT_A || fabs(c.b) > CURRENT_FAULT_A || fabs(ic) > CURRENT_FAULT_A) {
    haltOnFault("과전류 감지! (정지 상태로 전압 인가 시 역기전력이 없어 저항이 낮은 모터는 낮은 전압에서도 전류가 커질 수 있습니다. TEST_VOLTAGE를 더 낮추거나, 실제 쇼트가 아닌지 무전원 상태에서 상간 저항을 멀티미터로 확인하세요.)");
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // AS5047P SPI 센서 초기화
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);
  sensor.init();

  // 드라이버 설정
  driver.voltage_power_supply = 9.0;
  driver.voltage_limit = 5.0;
  if (!driver.init()) {
    haltOnFault("드라이버 초기화 실패!");
    return;
  }

  // 전류센서 초기화 (드라이버가 아직 disable 상태 = 무전류일 때 오프셋 자동 캘리브레이션됨)
  current_sense.linkDriver(&driver);
  if (!current_sense.init()) {
    haltOnFault("전류센서 초기화 실패!");
    return;
  }
  current_sense.skip_align = true;

  driver.enable();

  Serial.println(F("=== MKS ESP32 FOC Mega 자동 하드웨어 진단 테스트 시작 ==="));
  Serial.println(F("(중단하려면 시리얼 모니터에 x 입력)"));
  Serial.println(F("[Stage1] 6-step 벡터별 통전(전류) 확인 시작..."));
  delay(1000);
}

// ---------------- Stage 1: 통전 확인 ----------------
bool stage1_result[6];
int stage1_step = 0;

void runStage1() {
  if (userAbortRequested()) { haltOnFault("사용자 중단"); return; }

  if (stage1_step < 6) {
    float angle_el = stage1_step * (PI / 3.0f);
    setVector(TEST_VOLTAGE, angle_el);
    delay(SETTLE_MS);

    PhaseCurrent_s c = current_sense.readAverageCurrents(20);
    if (checkOvercurrent(c)) return;

    float ic = -(c.a + c.b);
    float mag = sqrt(c.a * c.a + c.b * c.b + ic * ic);
    bool pass = mag > CURRENT_THRESHOLD_A;
    stage1_result[stage1_step] = pass;

    Serial.print("  [Stage1] Step "); Serial.print(stage1_step);
    Serial.print(" angle="); Serial.print(angle_el * 180.0f / PI, 0);
    Serial.print("deg  Ia="); Serial.print(c.a, 3);
    Serial.print("A  Ib="); Serial.print(c.b, 3);
    Serial.print("A  Ic(추정)="); Serial.print(ic, 3);
    Serial.print("A  |I|="); Serial.print(mag, 3);
    Serial.println(pass ? "A  -> 통전 OK" : "A  -> 통전 없음 (FAIL)");

    if (STAGE1_HOLD_MS > SETTLE_MS) delay(STAGE1_HOLD_MS - SETTLE_MS);
    stage1_step++;
    return;
  }

  driver.setPwm(0, 0, 0);
  bool all_pass = true;
  for (int i = 0; i < 6; i++) if (!stage1_result[i]) all_pass = false;

  if (all_pass) {
    Serial.println(F("[Stage1] PASS: 6개 스텝 모두 통전 확인됨. Stage2로 진행합니다.\n"));
    stage = STAGE2_MOVEMENT;
  } else {
    haltOnFault("Stage1 실패: 일부 스텝에서 전류가 흐르지 않았습니다. 위 로그의 FAIL 스텝 기준으로 해당 상의 배선/FET을 점검하세요.");
  }
}

// ---------------- Stage 2: 구동력(움직임) 확인 ----------------
bool stage2_result[6];
int stage2_step = 0;

void runStage2() {
  if (userAbortRequested()) { haltOnFault("사용자 중단"); return; }

  if (stage2_step < 6) {
    float angle_el = stage2_step * (PI / 3.0f);

    sensor.update();
    float before = sensor.getAngle();

    setVector(TEST_VOLTAGE, angle_el);
    delay(STAGE2_HOLD_MS);

    sensor.update();
    float after = sensor.getAngle();

    PhaseCurrent_s c = current_sense.readAverageCurrents(20);
    if (checkOvercurrent(c)) return;

    float delta = after - before;
    bool pass = fabs(delta) > ANGLE_MOVE_THRESHOLD_RAD;
    stage2_result[stage2_step] = pass;

    Serial.print("  [Stage2] Step "); Serial.print(stage2_step);
    Serial.print(" angle_delta="); Serial.print(delta, 4);
    Serial.print("rad ("); Serial.print(delta * 180.0f / PI, 2);
    Serial.println(pass ? "deg)  -> 움직임 감지 OK" : "deg)  -> 움직임 없음");

    stage2_step++;
    return;
  }

  driver.setPwm(0, 0, 0);
  int pass_count = 0;
  for (int i = 0; i < 6; i++) if (stage2_result[i]) pass_count++;

  Serial.print(F("[Stage2] 결과: 6개 스텝 중 ")); Serial.print(pass_count); Serial.println(F("개 스텝에서 움직임 감지됨."));

  if (pass_count >= 4) {
    Serial.println(F("[Stage2] PASS. Stage3로 진행합니다.\n"));
    stage = STAGE3_DIRECTION;
  } else {
    haltOnFault("Stage2 실패: 전류는 흐르지만 축이 거의 움직이지 않습니다. 기계적 구속(축 고착) 또는 전압 부족을 의심하세요 (TEST_VOLTAGE 상향 시도).");
  }
}

// ---------------- Stage 3: 회전방향/상 순서 판별 ----------------
const int STAGE3_TOTAL_STEPS = 6 * STAGE3_CYCLES;
int stage3_idx = 0;
float stage3_cum_delta = 0;
int stage3_pos_count = 0, stage3_neg_count = 0;
float stage3_abs_delta_sum = 0;

void runStage3() {
  if (userAbortRequested()) { haltOnFault("사용자 중단"); return; }

  if (stage3_idx < STAGE3_TOTAL_STEPS) {
    int step_in_cycle = stage3_idx % 6;
    float angle_el = step_in_cycle * (PI / 3.0f);

    sensor.update();
    float before = sensor.getAngle();

    setVector(TEST_VOLTAGE, angle_el);
    delay(STAGE3_STEP_MS);

    sensor.update();
    float after = sensor.getAngle();

    PhaseCurrent_s c = current_sense.readAverageCurrents(10);
    if (checkOvercurrent(c)) return;

    float delta = after - before;
    stage3_cum_delta += delta;
    stage3_abs_delta_sum += fabs(delta);
    if (delta > 0.001f) stage3_pos_count++;
    else if (delta < -0.001f) stage3_neg_count++;

    stage3_idx++;
    return;
  }

  driver.setPwm(0, 0, 0);
  driver.disable();

  Serial.println(F("[Stage3] 연속 회전 테스트 완료."));
  Serial.print(F("  누적 각도 변화: ")); Serial.print(stage3_cum_delta, 3); Serial.println(F(" rad"));
  Serial.print(F("  정방향 스텝: ")); Serial.print(stage3_pos_count);
  Serial.print(F("  /  역방향 스텝: ")); Serial.println(stage3_neg_count);

  int dominant = max(stage3_pos_count, stage3_neg_count);
  if (dominant < STAGE3_TOTAL_STEPS * 0.7f) {
    haltOnFault("Stage3 실패: 회전 방향이 일관되지 않습니다. 극쌍수(PP) 설정이 실제 모터와 다르거나 배선 문제가 의심됩니다.");
    return;
  }

  if (stage3_cum_delta > 0) {
    Serial.println(F("[Stage3] PASS: 스텝 증가 방향(코드 기준 정방향)과 실제 회전 방향이 일치합니다."));
  } else {
    Serial.println(F("[Stage3] PASS (방향 반전): 실제 회전 방향이 스텝 증가 방향과 반대입니다."));
    Serial.println(F("  -> 이후 코드에서는 모터 3상 중 두 선을 서로 바꾸거나, motor.sensor_direction / DIR 값을 반전해서 사용하세요."));
  }

  // 참고용: 극쌍수(Pole Pairs) 추정치 (60도 전기각 스텝당 실제 이동한 기계각으로 역산)
  float avg_deg_per_step = (stage3_abs_delta_sum / STAGE3_TOTAL_STEPS) * 180.0f / PI;
  if (avg_deg_per_step > 0.01f) {
    int pp_estimate = round(60.0f / avg_deg_per_step);
    Serial.print(F("  참고: 극쌍수(Pole Pairs) 추정치 = "));
    Serial.println(pp_estimate);
  }

  Serial.println();
  stage = TEST_PASSED;
}

void loop() {
  switch (stage) {
    case STAGE1_CONTINUITY:
      runStage1();
      break;
    case STAGE2_MOVEMENT:
      runStage2();
      break;
    case STAGE3_DIRECTION:
      runStage3();
      break;
    case TEST_PASSED: {
      static bool printed = false;
      if (!printed) {
        Serial.println(F("=== 전체 테스트 통과: 하드웨어(인버터/배선/센서) 정상 동작 확인 ==="));
        printed = true;
      }
      break;
    }
    case TEST_FAILED:
      // 정지 상태 유지 (드라이버는 이미 disable됨)
      break;
  }
}
