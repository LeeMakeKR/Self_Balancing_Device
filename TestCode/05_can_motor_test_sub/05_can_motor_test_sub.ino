// CAN 모터 지령 테스트 - 서브 보드 (MKS ESP32 FOC Mega + AS5047P)
//
// 역할: 메인이 보내는 0x080 제어 명령을 받아 자기 축 목표 속도만 뽑아내고,
//       폐루프 속도제어로 모터를 돌린 뒤 0x09n 텔레메트리로 실측값을 되돌려 줍니다.
//
// 짝이 되는 스케치: 05_can_motor_test_main  (ESP32-S3 보드에 업로드)
// 참조 문서       : CAN_프로토콜.md
//
// ─────────────────────────────────────────────────────────────────────
// 개루프가 아니라 폐루프를 쓰는 이유
// ─────────────────────────────────────────────────────────────────────
//   밸런스휠이 달린 상태에서는 관성이 커서 개루프가 즉시 동기를 잃습니다
//   (01_direction_rpm_test 1단계 주석 참조). AS5047 폐루프는 01/02 테스트에서
//   이미 검증됐으므로 그대로 씁니다. 덕분에 "지령대로 도는지"를 실측으로 확인할 수 있습니다.
//
// ─────────────────────────────────────────────────────────────────────
// 배선
// ─────────────────────────────────────────────────────────────────────
//   CAN     : GPIO15(TX) -> 트랜시버 D,  GPIO4(RX) <- 트랜시버 R
//   Node ID : IO34(bit0), IO35(bit1) + 외부 10kΩ 풀업 각 1개
//             점퍼(GND)=0, 개방=1.  00->1축  01->2축  10->3축  11->오류
//   모터    : GPIO32/33/25 (3상), GPIO12 (EN)
//   AS5047  : SCLK 18, MISO 19, MOSI 23, CS 5
//   전류센싱: GPIO39, GPIO36 (INA240 인라인, 보드 실장)
//   두 보드 GND 공통 연결 필수. 버스 양 끝 120Ω 종단.
//
// ─────────────────────────────────────────────────────────────────────
// 안전
// ─────────────────────────────────────────────────────────────────────
//   - 메인 명령이 15ms 끊기면 스스로 모터를 끕니다 (CAN_프로토콜.md 6절 워치독).
//   - 이 전원 구성은 회생 전력을 흡수하지 못하므로 능동 감속을 쓰지 않습니다.
//     정지는 드라이버를 끄고 마찰로 코스팅합니다 (02_ramp_tuning과 동일 방침).
//   - 처음 돌릴 때는 휠을 떼거나 보드를 고정하세요.
//
// 시리얼 115200. 1초마다 상태 출력. 'x' 입력 시 즉시 정지.

#include <SimpleFOC.h>
#include "driver/twai.h"

// ===== 핀 (MKS ESP32 FOC Mega 실측) =====
#define PIN_A  32
#define PIN_B  33
#define PIN_C  25
#define PIN_EN 12

#define PIN_SPI_SCLK 18
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 23
#define PIN_SPI_CS    5

#define PIN_CAN_TX 15
#define PIN_CAN_RX  4

#define PIN_ID0 34   // bit0
#define PIN_ID1 35   // bit1

// ===== 전류 센싱 (INA240 인라인) =====
// MKS 공식 예제 07_current_control / 09_online_current_sense_test 기준값입니다.
//   InlineCurrentSense(0.01, 50.0, 39, 36)  = 션트 0.01Ω, 게인 50V/V, ADC 39/36
// GPIO39/36은 ADC1 채널(SENSOR_VN/VP)이라 WiFi와 충돌하지 않습니다.


#define PIN_CS_A 39
#define PIN_CS_B 36

const float SHUNT_OHM = 0.01f;
const float CS_GAIN   = 50.0f;

// ===== CAN ID (CAN_프로토콜.md 3절) =====
#define ID_ESTOP      0x010
#define ID_FAULT_BASE 0x020
#define ID_CMD        0x080
#define ID_TELEM_BASE 0x090

// ===== 폴트 코드 =====
#define FAULT_NONE        0x00
#define FAULT_CAN_WDT     0x06
#define FAULT_NODE_ID     0x08

// ===== 모터 파라미터 =====
// 01/02 테스트에서 확정된 값. initFOC이 "PP check" 경고를 내면 로그의 est. pp로 교체할 것.
#define POLE_PAIRS 10

const float SUPPLY_VOLTAGE = 12.0f;
const float VOLTAGE_LIMIT  = 11.0f;

// ===== 안전 한계 =====
// 메인도 ±30 rad/s로 제한하지만, 서브가 자기 한계를 따로 갖는 게 원칙입니다.
// 메인 펌웨어 버그가 그대로 모터로 전달되면 안 됩니다.
const float VEL_LIMIT_RADS   = 30.0f;   // 약 286 RPM
const float TARGET_SLEW_RADS2 = 20.0f;  // 목표값 변화율 상한. 계단 지령을 완만하게 만듭니다.
                                        // 휠 관성이 크면 계단 지령이 곧바로 최대 전류 요구가 됩니다.

// ===== 워치독 =====
const unsigned long CMD_TIMEOUT_MS = 15;   // 5ms 주기 기준 3프레임

// ===== 주기 =====
const unsigned long PRINT_PERIOD_MS = 1000;

// ===== 객체 =====
BLDCMotor          motor  = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM     driver = BLDCDriver3PWM(PIN_A, PIN_B, PIN_C, PIN_EN);
MagneticSensorSPI  sensor = MagneticSensorSPI(AS5047_SPI, PIN_SPI_CS);
InlineCurrentSense csense = InlineCurrentSense(SHUNT_OHM, CS_GAIN, PIN_CS_A, PIN_CS_B);

// ===== 상태 =====
uint8_t node_id = 0;              // 1..3
uint32_t id_telem = 0, id_fault = 0;

float   target_cmd  = 0.0f;       // 메인이 보낸 원본 목표 (rad/s)
float   target_slew = 0.0f;       // 슬루 제한을 거친 실제 지령
bool    want_enable = false;      // 메인의 enable 비트
bool    motor_on    = false;      // 실제 드라이버 상태
bool    estop_latch = false;      // E-STOP은 자동 해제하지 않습니다
bool    wdt_tripped = false;
uint8_t last_seq    = 0;

// 워치독이 걸렸다는 사실을 메인이 확실히 관측할 수 있게 하는 래치입니다.
// 워치독으로 멈춘 동안에는 메인 명령이 없으니 텔레메트리도 못 보냅니다.
// 그래서 "복구 직후" 몇 프레임에 WDT 비트를 남겨야 메인이 그 사건을 볼 수 있습니다.
// 이게 없으면 메인 쪽 보고서에 워치독이 영원히 "미관측"으로 남습니다.
#define WDT_REPORT_FRAMES 20     // 200Hz에서 100ms
uint8_t wdt_report_left = 0;

unsigned long last_cmd_ms   = 0;
unsigned long last_print_ms = 0;
unsigned long last_slew_us  = 0;

uint32_t rx_cmd_count = 0, tx_telem_count = 0, tx_fail = 0;

bool  csense_ok    = false;   // 전류 센싱 초기화 성공 여부
float current_filt = 0.0f;    // 전류 EMA (A)
float current_peak = 0.0f;    // 출력 주기 내 최댓값 (A)

// ─────────────────────────────────────────────────────────────────────
// Node ID (05_node_id_test와 동일 판정)
// ─────────────────────────────────────────────────────────────────────
#define ID_SAMPLE_N 10

// 실패 시 0 반환
uint8_t readNodeId() {
  int c0 = 0, c1 = 0;
  for (int i = 0; i < ID_SAMPLE_N; i++) {
    c0 += digitalRead(PIN_ID0);
    c1 += digitalRead(PIN_ID1);
    delay(2);
  }

  // 10회가 전부 일치해야 확정합니다. 갈리면 풀업 미연결/접촉 불량입니다.
  if (c0 != 0 && c0 != ID_SAMPLE_N) return 0;
  if (c1 != 0 && c1 != ID_SAMPLE_N) return 0;

  int b0 = (c0 == ID_SAMPLE_N) ? 1 : 0;
  int b1 = (c1 == ID_SAMPLE_N) ? 1 : 0;

  switch ((b1 << 1) | b0) {
    case 0b00: return 1;
    case 0b01: return 2;
    case 0b10: return 3;
    default:   return 0;   // 0b11 = 점퍼 전부 빠짐
  }
}

// ─────────────────────────────────────────────────────────────────────
// CAN
// ─────────────────────────────────────────────────────────────────────
bool canInit() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
  g.tx_queue_len = 20;
  g.rx_queue_len = 20;

  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    Serial.println(F("[FAIL] twai_driver_install"));
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println(F("[FAIL] twai_start"));
    return false;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────
// 전류
// ─────────────────────────────────────────────────────────────────────
// FOC 루프마다 ADC를 읽으면 analogRead 2회(약 20us)가 매 루프에 얹혀 루프율이 떨어집니다.
// 텔레메트리가 200Hz이므로 1kHz 샘플링이면 충분합니다.
const unsigned long CURRENT_SAMPLE_US = 1000;

// PWM에 동기화된 샘플링이 아니라 원시값은 상당히 튑니다.
// 텔레메트리에는 EMA를 실어 보내고, 순간 최댓값은 시리얼로 따로 봅니다.
const float CURRENT_EMA_ALPHA = 0.2f;

void updateCurrent() {
  if (!csense_ok) return;

  static unsigned long last_us = 0;
  unsigned long now = micros();
  if (now - last_us < CURRENT_SAMPLE_US) return;
  last_us = now;

  // 전기각을 넘기면 q축 방향으로 투영된 부호 있는 전류가 나옵니다.
  // 부호로 역행(제동) 구간을 구분할 수 있어 크기만 보는 것보다 낫습니다.
  float amps = csense.getDCCurrent(motor.electrical_angle);

  current_filt += CURRENT_EMA_ALPHA * (amps - current_filt);
  if (fabsf(amps) > current_peak) current_peak = fabsf(amps);
}

void sendTelemetry(uint8_t seq_echo) {
  twai_message_t m;
  m.flags = 0;
  m.identifier = id_telem;
  m.data_length_code = 8;

  // 각도는 스케일 변환 없이 14bit 원본으로 보냅니다 (CAN_프로토콜.md 4.2).
  float mech = sensor.getMechanicalAngle();
  if (mech < 0) mech += _2PI;
  uint16_t angle_raw = (uint16_t)(mech / _2PI * 16384.0f) & 0x3FFF;

  float vel = motor.shaft_velocity * 10.0f;      // rad/s x10
  if (vel >  32767.0f) vel =  32767.0f;
  if (vel < -32768.0f) vel = -32768.0f;
  int16_t velocity = (int16_t)vel;

  // INA240 인라인 전류 센싱 실측값 (mA).
  // 초기화에 실패했으면 추정값을 만들어 넣지 않고 0을 보냅니다.
  // 규격상 자리를 임의 값으로 채우면 나중에 진짜 이상을 못 잡습니다.
  int16_t current_ma = 0;
  if (csense_ok) {
    float ma = current_filt * 1000.0f;
    if (ma >  32767.0f) ma =  32767.0f;
    if (ma < -32768.0f) ma = -32768.0f;
    current_ma = (int16_t)ma;
  }

  uint8_t status = 0;
  if (motor_on)    status |= 0x01;   // bit0 enable
  if (estop_latch) status |= 0x02;   // bit1 fault
  status |= 0x04;                    // bit2 initFOC 정렬 완료

  // bit3 워치독. 현재 걸려 있는 동안은 물론, 복구 직후 일정 프레임 동안도 유지합니다.
  if (wdt_tripped || wdt_report_left) {
    status |= 0x08;
    if (wdt_report_left) wdt_report_left--;
  }

  m.data[0] = seq_echo;
  m.data[1] = status;
  m.data[2] = angle_raw & 0xFF;   m.data[3] = (angle_raw >> 8) & 0xFF;
  m.data[4] = velocity  & 0xFF;   m.data[5] = (velocity  >> 8) & 0xFF;
  m.data[6] = current_ma & 0xFF;  m.data[7] = (current_ma >> 8) & 0xFF;

  if (twai_transmit(&m, 0) == ESP_OK) tx_telem_count++;
  else                                tx_fail++;
}

void sendFault(uint8_t code, uint8_t severity, int16_t value) {
  twai_message_t m;
  m.flags = 0;
  m.identifier = id_fault ? id_fault : (ID_FAULT_BASE + 1);
  m.data_length_code = 8;

  uint32_t ts = millis();
  m.data[0] = code;
  m.data[1] = severity;
  m.data[2] = value & 0xFF;  m.data[3] = (value >> 8) & 0xFF;
  m.data[4] = ts & 0xFF;         m.data[5] = (ts >> 8) & 0xFF;
  m.data[6] = (ts >> 16) & 0xFF; m.data[7] = (ts >> 24) & 0xFF;

  twai_transmit(&m, 0);
}

void handleCan() {
  twai_message_t m;
  // 큐를 전부 비웁니다. 남겨두면 지연이 누적됩니다.
  while (twai_receive(&m, 0) == ESP_OK) {

    if (m.identifier == ID_ESTOP) {
      estop_latch = true;
      Serial.println(F("!! E-STOP 수신 - 모터 차단 (전원 재인가 전까지 유지)"));
      continue;
    }

    if (m.identifier != ID_CMD || m.data_length_code != 8) continue;

    last_seq = m.data[0];
    uint8_t flags = m.data[1];

    // 자기 축 목표값만 꺼냅니다. node_id 1 -> data[2..3], 2 -> [4..5], 3 -> [6..7]
    int off = 2 + (node_id - 1) * 2;
    int16_t raw = (int16_t)((uint16_t)m.data[off] | ((uint16_t)m.data[off + 1] << 8));

    float t = raw / 10.0f;                        // rad/s
    if (t >  VEL_LIMIT_RADS) t =  VEL_LIMIT_RADS; // 서브 자체 한계로 한 번 더 자릅니다
    if (t < -VEL_LIMIT_RADS) t = -VEL_LIMIT_RADS;
    target_cmd = t;

    bool axis_enable = (flags >> (node_id - 1)) & 0x01;
    bool brake       = (flags >> 3) & 0x01;
    want_enable = axis_enable && !brake && !estop_latch;

    last_cmd_ms = millis();
    if (wdt_tripped) {
      wdt_tripped = false;
      Serial.println(F("워치독 해제 - 명령 수신 재개"));
    }
    rx_cmd_count++;

    // 받은 즉시 되돌려 보냅니다. 왕복 지연이 프레임 단위로 정확히 측정됩니다.
    sendTelemetry(last_seq);
  }
}

// ─────────────────────────────────────────────────────────────────────
// 모터
// ─────────────────────────────────────────────────────────────────────
void setMotorOn(bool on) {
  if (on == motor_on) return;

  if (on) {
    // 이전 구동의 적분항이 남아 있으면 켜자마자 튑니다 (02_ramp_tuning 163행 주석).
    motor.PID_velocity.reset();
    target_slew = motor.shaft_velocity;   // 현재 실속도에서 이어받아 계단 지령 방지
    motor.enable();
  } else {
    motor.disable();                      // 능동 감속 없이 코스팅
    target_slew = 0.0f;
  }
  motor_on = on;
}

void updateSlew() {
  unsigned long now = micros();
  float dt = (now - last_slew_us) * 1e-6f;
  last_slew_us = now;
  if (dt <= 0.0f || dt > 0.1f) return;    // 첫 호출/이상값 보호

  float step = TARGET_SLEW_RADS2 * dt;
  float diff = target_cmd - target_slew;

  if (diff >  step) target_slew += step;
  else if (diff < -step) target_slew -= step;
  else target_slew = target_cmd;
}

void checkWatchdog() {
  if (wdt_tripped) return;
  if (millis() - last_cmd_ms < CMD_TIMEOUT_MS) return;

  wdt_tripped = true;
  wdt_report_left = WDT_REPORT_FRAMES;   // 복구 후에도 메인이 볼 수 있게 래치
  target_cmd  = 0.0f;
  want_enable = false;
  setMotorOn(false);

  Serial.println(F("!! CAN 워치독 타임아웃 - 모터 차단"));
  sendFault(FAULT_CAN_WDT, 1, 0);
}

// ─────────────────────────────────────────────────────────────────────
void printStatus() {
  Serial.print(F("node ")); Serial.print(node_id);
  Serial.print(F("  cmd "));   Serial.print(target_cmd, 1);
  Serial.print(F("  slew "));  Serial.print(target_slew, 1);
  Serial.print(F("  actual ")); Serial.print(motor.shaft_velocity, 1);
  Serial.print(F(" rad/s"));
  Serial.print(F("  Vq "));    Serial.print(motor.voltage.q, 2);
  Serial.print(F("V"));
  if (csense_ok) {
    Serial.print(F("  I ")); Serial.print(current_filt * 1000.0f, 0);
    Serial.print(F("mA (peak ")); Serial.print(current_peak * 1000.0f, 0);
    Serial.print(F(")"));
  } else {
    Serial.print(F("  I --"));
  }
  Serial.print(F("  "));
  Serial.print(motor_on ? F("ON ") : F("off "));
  if (wdt_tripped) Serial.print(F("WDT "));
  if (estop_latch) Serial.print(F("ESTOP "));
  Serial.print(F("  rx ")); Serial.print(rx_cmd_count);
  Serial.print(F("  tx ")); Serial.print(tx_telem_count);
  if (tx_fail) { Serial.print(F("  txfail ")); Serial.print(tx_fail); }
  Serial.println();

  rx_cmd_count = 0; tx_telem_count = 0; tx_fail = 0;
  current_peak = 0.0f;
}

void handleSerial() {
  while (Serial.available()) {
    if (Serial.read() == 'x') {
      estop_latch = true;
      target_cmd = 0.0f;
      want_enable = false;
      setMotorOn(false);
      Serial.println(F("-> 수동 정지 (리셋 전까지 유지)"));
    }
  }
}

// ─────────────────────────────────────────────────────────────────────
// 모터 정지 확인
// ─────────────────────────────────────────────────────────────────────
// initFOC 정렬은 모터를 실제로 돌립니다. 휠 관성 때문에 정렬이 끝나도
// 한동안 코스팅합니다. 고정 시간을 기다리는 방식은 휠 무게나 마찰이 바뀌면
// 그대로 틀어지므로, 자기각 센서로 "실제로 멈췄는지"를 확인합니다.
//
// 순간 속도(sensor.getVelocity())를 쓰지 않는 이유:
// 14bit 양자화 노이즈 때문에 아직 돌고 있어도 순간값 0이 튀어나옵니다.
// 고정 시간창 동안의 각도 변화량으로 계산해야 오판이 없습니다.
// (02_ramp_tuning의 코스팅 정지 판정과 같은 방식)
const float         STOP_VEL_RADS      = 0.05f;   // 이 아래면 정지 후보
const int           STOP_CONFIRM       = 3;       // 연속 몇 창을 만족해야 확정
const unsigned long STOP_WINDOW_MS     = 200;     // 속도 계산 시간창
const unsigned long STOP_TIMEOUT_MS    = 30000;   // 안전장치
const unsigned long SETTLE_AFTER_STOP_MS = 3000;  // 정지 확인 후 대기

void waitMotorStopped() {
  Serial.println(F("모터 정지 확인 중 (자기각 센서 기준)..."));

  unsigned long t_begin = millis();
  int confirms = 0;

  while (true) {
    sensor.update();
    float         a0 = sensor.getAngle();     // 회전수 포함 누적 각도
    unsigned long w0 = millis();

    // 창이 도는 동안에도 update()를 계속 호출해야 회전수 카운트가 어긋나지 않습니다.
    while (millis() - w0 < STOP_WINDOW_MS) {
      sensor.update();
      delay(1);
    }

    float vel = (sensor.getAngle() - a0) / (STOP_WINDOW_MS / 1000.0f);

    if (fabsf(vel) < STOP_VEL_RADS) confirms++;
    else                            confirms = 0;

    Serial.print(F("  vel ")); Serial.print(vel, 3);
    Serial.print(F(" rad/s   확인 ")); Serial.print(confirms);
    Serial.print('/'); Serial.println(STOP_CONFIRM);

    if (confirms >= STOP_CONFIRM) break;

    if (millis() - t_begin > STOP_TIMEOUT_MS) {
      Serial.println(F("[WARN] 정지 확인 시간 초과. 그대로 진행합니다."));
      Serial.println(F("       휠이 계속 돌고 있다면 기계적 원인을 확인하세요."));
      break;
    }
  }

  Serial.print(F("정지 확인. ")); Serial.print(SETTLE_AFTER_STOP_MS / 1000);
  Serial.println(F("초 대기 후 시작합니다."));

  unsigned long t = millis();
  while (millis() - t < SETTLE_AFTER_STOP_MS) {
    sensor.update();
    delay(1);
  }
}

// ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("=========================================="));
  Serial.println(F(" CAN motor command test - SUB (ESP32 FOC)"));
  Serial.println(F("=========================================="));

  // --- Node ID ---
  // INPUT_PULLUP 금지. GPIO34~39에는 내부 풀업이 없어 조용히 무시됩니다.
  pinMode(PIN_ID0, INPUT);
  pinMode(PIN_ID1, INPUT);
  node_id = readNodeId();

  if (node_id == 0) {
    Serial.println(F("[FAIL] Node ID 판정 실패."));
    Serial.println(F("  점퍼가 전부 빠졌거나 10kΩ 풀업이 연결되지 않았습니다."));
    Serial.println(F("  1축: IO34+IO35 둘 다 / 2축: IO35만 / 3축: IO34만"));
    Serial.println(F("  05_node_id_test로 먼저 확인하세요. 모터는 기동하지 않습니다."));
    while (1) delay(500);
  }

  id_telem = ID_TELEM_BASE + node_id;
  id_fault = ID_FAULT_BASE + node_id;

  Serial.print(F("Node ID ")); Serial.print(node_id);
  Serial.print(F(" (")); Serial.print(node_id); Serial.print(F("번축)  telem 0x"));
  Serial.print(id_telem, HEX);
  Serial.print(F("  cmd offset data[")); Serial.print(2 + (node_id - 1) * 2);
  Serial.println(F("..]"));

  // --- CAN ---
  if (!canInit()) { while (1) delay(100); }
  Serial.print(F("CAN ready: TX=GPIO")); Serial.print(PIN_CAN_TX);
  Serial.print(F(" RX=GPIO")); Serial.println(PIN_CAN_RX);

  // --- 센서 ---
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);
  sensor.clock_speed = 10000000;   // 기본 1MHz는 고속에서 병목
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

  // --- 전류 센싱 (INA240 인라인) ---
  // MKS 예제 07은 gain_a / gain_b 를 모두 반전시킵니다 (예제 09는 gain_b만 반전).
  // 두 예제가 어긋나므로, 실제로 FOC 전류제어까지 돌린 07 쪽을 따릅니다.
  // 전류 부호가 회전 방향과 반대로 보이면 아래 두 줄을 조정하세요.
  //
  // skip_align = true 인 이유: initFOC의 전류센스 정렬이 실패하면 motor_status가
  // motor_ready가 아니게 되어 아래 검사에서 걸립니다. 이 스케치의 목적은 CAN 지령
  // 검증이므로, 제조사가 검증한 게인 부호를 그대로 쓰고 정렬 단계는 건너뜁니다.
  if (csense.init()) {
    csense.gain_a *= -1;
    csense.gain_b *= -1;
    csense.skip_align = true;
    motor.linkCurrentSense(&csense);
    csense_ok = true;
    Serial.print(F("current sense OK: shunt "));
    Serial.print(SHUNT_OHM, 3); Serial.print(F("ohm, gain "));
    Serial.print(CS_GAIN, 0);   Serial.print(F(", ADC "));
    Serial.print(PIN_CS_A); Serial.print('/'); Serial.println(PIN_CS_B);
  } else {
    csense_ok = false;
    Serial.println(F("[WARN] current sense init 실패 - 전류는 0으로 보고합니다."));
    Serial.println(F("       모터 제어 자체는 voltage 모드라 계속 진행됩니다."));
  }

  // --- 폐루프 속도제어 (02_ramp_tuning에서 검증된 설정) ---
  motor.controller        = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation    = FOCModulationType::SpaceVectorPWM;
  motor.voltage_limit     = VOLTAGE_LIMIT;
  motor.velocity_limit    = VEL_LIMIT_RADS;

  motor.PID_velocity.P           = 0.1f;
  motor.PID_velocity.I           = 1.0f;
  motor.PID_velocity.D           = 0.0f;
  motor.PID_velocity.output_ramp = 200.0f;   // 02번과 달리 여기서는 출력단도 제한합니다.
                                             // 램프율을 재는 테스트가 아니라 실사용 조건이라
                                             // 계단 지령에서 전류가 튀는 것을 막는 쪽이 우선입니다.
  motor.LPF_velocity.Tf          = 0.02f;

  motor.init();
  motor.initFOC();

  if (motor.motor_status != FOCMotorStatus::motor_ready) {
    Serial.println(F("[FAIL] initFOC. POLE_PAIRS와 센서 배선을 확인하세요."));
    while (1) delay(100);
  }

  motor.disable();          // 메인이 enable 할 때까지 꺼둡니다
  motor_on = false;

  // initFOC 정렬로 돌던 휠이 완전히 멈춘 것을 센서로 확인한 뒤 진행합니다.
  // 이게 끝나야 메인에 텔레메트리를 보내기 시작하므로, 메인은 자동으로
  // 안정된 상태에서 측정을 시작하게 됩니다.
  waitMotorStopped();

  // 정지 확인 동안 메인이 계속 명령을 보냈다면 수신 큐(20칸)가 차 있습니다.
  // 비우지 않으면 loop 진입 직후 20프레임에 한꺼번에 응답해 버스가 튑니다.
  twai_message_t dump;
  int drained = 0;
  while (twai_receive(&dump, 0) == ESP_OK) drained++;
  if (drained) { Serial.print(F("수신 큐 배수 ")); Serial.print(drained); Serial.println(F("프레임")); }

  Serial.println(F("initFOC OK. 메인의 enable 명령 대기 중."));
  Serial.println(F("'x' 입력 시 즉시 정지."));

  last_cmd_ms   = millis();
  last_print_ms = millis();
  last_slew_us  = micros();
  wdt_tripped   = true;     // 첫 명령을 받기 전까지는 정지 상태로 둡니다
}

void loop() {
  handleSerial();
  handleCan();
  checkWatchdog();

  // enable 상태 반영
  setMotorOn(want_enable && !wdt_tripped && !estop_latch);

  updateSlew();
  updateCurrent();

  motor.loopFOC();
  motor.move(motor_on ? target_slew : 0.0f);

  if (millis() - last_print_ms >= PRINT_PERIOD_MS) {
    last_print_ms = millis();
    printStatus();
  }
}
