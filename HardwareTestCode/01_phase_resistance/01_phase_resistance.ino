// 모터 상저항(phase resistance) 측정 (MKS ESP32 FOC Mega + INA240 인라인 전류센스)
//
// [목적]
//   SimpleFOC의 motor.phase_resistance 에 넣을 "상(相)당 저항 R" 을 실측합니다.
//   이 값이 있어야 SimpleFOC가 전압 지령을 전류로 환산할 수 있고(current_limit,
//   voltage_limit 자동 계산), 토크 상수 기반 제어와 KV 추정이 의미를 가집니다.
//
// [측정 원리]
//   회전자를 세운 채(= 역기전력 0) 전기각 0도에 직류 Ud 만 인가합니다.
//   SinePWM 기준 상전압은  Ua = Ud,  Ub = Uc = -Ud/2  가 되므로
//     - 전류 경로 저항 = Ra + (Rb // Rc) = R + R/2 = 1.5R
//     - 경로에 걸린 전압 = Ua - Ubc     = Ud + Ud/2 = 1.5Ud
//     - I_a = 1.5Ud / 1.5R = Ud / R
//   따라서  R(상당 저항) = Ud / I_a  로 곧바로 나옵니다.
//   (Y결선 모터를 멀티미터로 상-상 측정하면 2R 이 나옵니다. 헷갈리지 말 것)
//
// [왜 여러 전압으로 재고 직선 피팅을 하는가]
//   측정된 저항에는 권선 저항뿐 아니라 MOSFET Rds(on), 배선/커넥터 저항,
//   전류센스 오프셋 같은 것들이 섞여 들어옵니다.
//   V-I 점 여러 개를 찍어 직선 피팅하면
//     - 기울기      = 저항 성분 (우리가 원하는 값)
//     - y절편(offset) = 전류와 무관한 전압 오차 성분
//   으로 분리되어, 단일 점 측정보다 신뢰할 수 있는 값이 나옵니다.
//   절편이 크게 나오면 전류센스 오프셋 보정이 틀어졌다는 신호입니다.
//
// [주의]
//   - 회전자가 멈춰 있어야 합니다(역기전력이 있으면 전부 틀어짐). 정지 상태에서 시작할 것.
//   - 이 모터는 권선 저항이 낮아 1~3V만 걸어도 정지 상태에서 수 A가 흐릅니다.
//     그래서 시험 전압을 1.3V 이하로 제한하고 전체 인가 시간을 수 초로 짧게 잡았습니다.
//   - 구리 저항은 온도에 비례해 커집니다(약 +0.4%/도). 차가운 상태에서 측정하고,
//     반복 측정 시에는 사이에 식힐 시간을 두세요.
//   - 극쌍수(POLE_PAIRS)는 이 측정에 영향을 주지 않습니다. 전기각을 직접 고정하기 때문입니다.
//
// [사용법]
//   업로드 후 시리얼 모니터 115200. setup()에서 측정이 자동으로 끝나고 결과를 출력합니다.
//   다시 측정하려면 보드를 리셋하세요.

#include <SimpleFOC.h>

// ===== 핀 정의 (MKS ESP32 FOC Mega 실측) =====
#define PIN_A  32
#define PIN_B  33
#define PIN_C  25
#define PIN_EN 12

// 인라인 전류센스 (M0 채널): 션트 0.01옴, INA240 게인 50배, ADC = A상 39 / B상 36
#define PIN_CS_A   39
#define PIN_CS_B   36
#define SHUNT_OHM  0.01f
#define AMP_GAIN   50.0f

// ===== 전원 / 안전 한계 =====
const float SUPPLY_VOLTAGE  = 12.0f;  // 실제 공급 전압에 맞출 것
const float DRIVER_V_LIMIT  = 3.0f;   // 이 스케치는 저전압만 쓰므로 상한을 낮게 잠가둡니다
const float CURRENT_ABORT   = 4.0f;   // 이 이상 흐르면 즉시 중단 (권선 보호)

// ===== 시험 조건 =====
// 오름차순. 너무 낮으면 ADC 분해능에, 너무 높으면 발열에 걸립니다.
const float TEST_V[]  = { 0.3f, 0.5f, 0.7f, 0.9f, 1.1f, 1.3f };
const int   N_POINT   = sizeof(TEST_V) / sizeof(TEST_V[0]);

const float         ALIGN_V   = 1.0f;   // 회전자를 전기각 0에 끌어다 붙이는 전압
const unsigned long ALIGN_MS  = 1500;   // 정렬 후 진동이 잦아들 때까지 대기
const unsigned long SETTLE_MS = 250;    // 각 시험 전압 인가 후 안정화 대기
const int           SAMPLES   = 300;    // 점당 평균할 ADC 샘플 수 (PWM 리플 평균화)

// ===== 객체 =====
// POLE_PAIRS 값은 결과에 영향 없음 (전기각을 직접 지정하므로)
BLDCMotor          motor         = BLDCMotor(10);
BLDCDriver3PWM     driver        = BLDCDriver3PWM(PIN_A, PIN_B, PIN_C, PIN_EN);
InlineCurrentSense current_sense = InlineCurrentSense(SHUNT_OHM, AMP_GAIN, PIN_CS_A, PIN_CS_B);

bool aborted = false;

// 지정한 Ud를 전기각 0에 걸고, A상 전류를 SAMPLES회 평균해서 반환.
// 과전류가 감지되면 드라이버를 끄고 aborted 플래그를 세웁니다.
float measureCurrent(float ud) {
  motor.setPhaseVoltage(0, ud, 0);   // Uq=0, Ud=ud, 전기각=0 -> 순수 직류 정렬 벡터
  delay(SETTLE_MS);

  float sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    PhaseCurrent_s c = current_sense.getPhaseCurrents();
    if (fabs(c.a) > CURRENT_ABORT) {
      motor.setPhaseVoltage(0, 0, 0);
      motor.disable();
      aborted = true;
      Serial.print(F("\n[ABORT] Overcurrent: "));
      Serial.print(c.a, 2);
      Serial.println(F(" A. Lower TEST_V or check wiring."));
      return 0;
    }
    sum += c.a;
    delayMicroseconds(200);
  }
  return sum / SAMPLES;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  SimpleFOCDebug::enable(&Serial);

  Serial.println(F("=========================================="));
  Serial.println(F(" Phase Resistance Measurement"));
  Serial.println(F(" Keep the rotor free but at rest before start"));
  Serial.println(F("=========================================="));

  // --- 드라이버 ---
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit        = DRIVER_V_LIMIT;
  if (!driver.init()) {
    Serial.println(F("[FAIL] Driver init failed. Check power supply and pin wiring."));
    aborted = true;
    return;
  }
  motor.linkDriver(&driver);

  // --- 모터 (센서 없음, initFOC 하지 않음) ---
  // 이 측정은 회전자 위치를 알 필요가 없습니다. 전기각을 우리가 직접 고정합니다.
  motor.voltage_limit  = DRIVER_V_LIMIT;
  motor.foc_modulation = FOCModulationType::SinePWM;  // 위 계산식이 그대로 성립하는 변조 방식
  motor.init();

  // --- 전류센스 ---
  // init()이 무전류 상태에서 ADC 오프셋을 자동 보정합니다.
  // 반드시 전압을 걸기 "전에" 호출해야 합니다.
  current_sense.linkDriver(&driver);
  if (!current_sense.init()) {
    Serial.println(F("[FAIL] Current sense init failed. Check ADC pins 39/36."));
    aborted = true;
    return;
  }
  current_sense.gain_b *= -1;   // MKS 보드 B상 션트 극성 반전 (제조사 예제와 동일)

  motor.enable();

  // --- 1) 회전자 정렬 ---
  // 전기각 0 벡터로 회전자를 끌어다 고정시킵니다. 진동이 남아 있으면
  // 역기전력이 섞여 저항이 실제보다 높거나 낮게 나옵니다.
  Serial.println(F("\nAligning rotor to electrical angle 0 ..."));
  motor.setPhaseVoltage(0, ALIGN_V, 0);
  delay(ALIGN_MS);

  // --- 2) 전압별 전류 측정 ---
  Serial.println(F("\nUd[V]\tIa[A]\tR=Ud/Ia[ohm]"));
  float v[N_POINT], i[N_POINT];
  int   n = 0;

  for (int k = 0; k < N_POINT; k++) {
    float ia = measureCurrent(TEST_V[k]);
    if (aborted) break;

    v[n] = TEST_V[k];
    i[n] = ia;
    n++;

    Serial.print(TEST_V[k], 2);
    Serial.print('\t');
    Serial.print(ia, 3);
    Serial.print('\t');
    if (fabs(ia) > 0.02f) Serial.println(TEST_V[k] / ia, 4);
    else                  Serial.println(F("-- (too small)"));
  }

  // 측정 끝. 권선 발열을 막기 위해 즉시 전압을 내리고 드라이버를 끕니다.
  motor.setPhaseVoltage(0, 0, 0);
  motor.disable();

  if (aborted || n < 2) {
    Serial.println(F("\n[FAIL] Not enough valid points."));
    Serial.println(F("       Check: 12V supply, EN pin (GPIO12), motor phase wires,"));
    Serial.println(F("              current sense pins 39/36."));
    return;
  }

  // --- 3) 최소자승 직선 피팅: V = R * I + offset ---
  float sum_i = 0, sum_v = 0;
  for (int k = 0; k < n; k++) { sum_i += i[k]; sum_v += v[k]; }
  float mean_i = sum_i / n, mean_v = sum_v / n;

  float num = 0, den = 0;
  for (int k = 0; k < n; k++) {
    num += (i[k] - mean_i) * (v[k] - mean_v);
    den += (i[k] - mean_i) * (i[k] - mean_i);
  }

  if (den < 1e-9f) {
    Serial.println(F("\n[FAIL] Current did not change with voltage. Sense or wiring problem."));
    return;
  }

  float R_slope  = num / den;                 // 기울기 = 저항 성분
  float V_offset = mean_v - R_slope * mean_i; // 절편 = 전류 무관 전압 오차

  Serial.println(F("\n=========================================="));
  Serial.println(F(" RESULT"));
  Serial.println(F("=========================================="));
  Serial.print(F("Phase resistance (fit slope) : "));
  Serial.print(R_slope, 4);
  Serial.println(F(" ohm  <- use this for motor.phase_resistance"));
  Serial.print(F("Fit offset voltage           : "));
  Serial.print(V_offset, 4);
  Serial.println(F(" V"));
  Serial.println();
  Serial.println(F("Notes:"));
  Serial.println(F(" - This is PER-PHASE R. A multimeter across two phase wires"));
  Serial.println(F("   of a wye motor reads about 2x this value."));
  Serial.println(F(" - Includes MOSFET Rds(on) and wiring, so it is slightly higher"));
  Serial.println(F("   than the pure winding resistance."));
  Serial.println(F(" - If the offset voltage is large (> ~0.1 V), suspect current"));
  Serial.println(F("   sense offset calibration - re-run with the rotor at rest."));
  Serial.println(F(" - Copper resistance rises ~0.4%/degC. Measure cold, and let the"));
  Serial.println(F("   motor cool between runs."));
  Serial.println(F("\nDriver disabled. Reset the board to measure again."));
}

void loop() {
  // 측정은 setup()에서 1회만 수행합니다. 여기서 전압을 계속 걸면 권선이 과열됩니다.
}
