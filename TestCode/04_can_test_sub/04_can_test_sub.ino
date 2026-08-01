// CAN 통신 테스트 - 서브 보드 (MKS ESP32 FOC Mega / ESP32-WROOM-32)
//
// 역할: 슬레이브. 메인 보드의 제어 명령(0x100)을 받아 즉시 텔레메트리(0x200)로
//       응답합니다. 이 스케치는 통신만 검증하며 모터는 전혀 건드리지 않습니다.
//
// 짝이 되는 스케치: 04_can_test_main  (ESP32-S3 메인 보드에 업로드)
//
// ─────────────────────────────────────────────────────────────────────
// 배선 (SN65HVD230 트랜시버, 양쪽 보드 동일)
// ─────────────────────────────────────────────────────────────────────
//   MCU TX  -> 트랜시버 D  (또는 TXD)
//   MCU RX  -> 트랜시버 R  (또는 RXD)
//   VCC     -> 3.3V        (SN65HVD230은 3.3V 소자. 5V 넣지 말 것)
//   GND     -> GND
//   RS      -> GND 직결    (고속 모드. VCC로 당기면 저전력 대기 모드가 되어
//                          송신이 죽습니다. 데이터시트 확인 완료)
//   CANH    -> 상대 보드 CANH
//   CANL    -> 상대 보드 CANL
//
//   [필수] 두 보드의 GND를 반드시 연결할 것.
//   [필수] 버스 양 끝에 120Ω 종단저항 (2노드면 양쪽에 하나씩).
//
// ─────────────────────────────────────────────────────────────────────
// 이 보드의 CAN 핀
// ─────────────────────────────────────────────────────────────────────
//   RX = GPIO4,  TX = GPIO15   (보드 배정 확정값)
//
//   기존 사용 핀과 충돌 없음:
//     모터 PWM 32/33/25,  드라이버 EN 12,
//     AS5047P SPI  CS 5 / SCLK 18 / MISO 19 / MOSI 23,
//     전류센싱 ADC 36/39
//   따라서 나중에 모터 제어와 CAN을 한 펌웨어로 합쳐도 그대로 쓸 수 있습니다.
//
//   참고: GPIO15는 ESP32 스트래핑 핀(MTDO)입니다. 다만 CAN TX는 유휴 상태에서
//   HIGH(recessive)이고 부팅 시점에는 내부 풀업으로 HIGH가 유지되므로
//   부팅 모드에 영향을 주지 않습니다.
//
// ─────────────────────────────────────────────────────────────────────
// 프로토콜 (8바이트 고정) — main 쪽과 동일
// ─────────────────────────────────────────────────────────────────────
//   0x100  MAIN -> SUB   제어 명령
//     [0..3] uint32  seq        시퀀스 번호
//     [4..5] int16   target_rpm 목표 속도
//     [6]    uint8   enable     모터 활성 플래그
//     [7]    uint8   checksum   0..6 바이트 XOR
//
//   0x200  SUB -> MAIN   텔레메트리
//     [0..3] uint32  seq        받은 명령의 seq 그대로 반향
//     [4..5] int16   actual_rpm 실측 속도 (지금은 모의값)
//     [6]    uint8   status     서브 상태 코드
//     [7]    uint8   checksum   0..6 바이트 XOR
//
// 시리얼 115200. 1초마다 통계 출력.

#include "driver/twai.h"

// ===== 핀 (ESP32 FOC 보드 CAN 배정) =====
#define PIN_CAN_TX 15
#define PIN_CAN_RX 4

// ===== CAN ID =====
#define ID_CMD   0x100   // MAIN -> SUB
#define ID_TELEM 0x200   // SUB  -> MAIN

// ===== 상태 코드 =====
#define ST_IDLE  0
#define ST_READY 1

const unsigned long STAT_PERIOD_MS = 1000;

// ===== 통계 =====
uint32_t stat_recv = 0, stat_sent = 0, stat_bad_sum = 0, stat_tx_fail = 0;
uint32_t last_seq = 0;
bool     seq_seen = false;
uint32_t stat_seq_gap = 0;      // 메인->서브 구간에서 빠진 프레임 수

// 마지막으로 받은 명령 내용 (표시용)
int16_t  last_target_rpm = 0;
uint8_t  last_enable = 0;

unsigned long last_stat_ms = 0;

uint8_t xorSum(const uint8_t* d, int n) {
  uint8_t s = 0;
  for (int i = 0; i < n; i++) s ^= d[i];
  return s;
}

const __FlashStringHelper* stateName(twai_state_t s) {
  switch (s) {
    case TWAI_STATE_STOPPED:    return F("STOPPED");
    case TWAI_STATE_RUNNING:    return F("RUNNING");
    case TWAI_STATE_BUS_OFF:    return F("BUS_OFF");
    case TWAI_STATE_RECOVERING: return F("RECOVERING");
    default:                    return F("UNKNOWN");
  }
}

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

// 이 테스트에서는 모터를 돌리지 않으므로 실측 대신 모의값을 만듭니다.
// 실제 구현에서는 motor.shaft_velocity 를 RPM으로 변환해 넣으면 됩니다.
int16_t makeActualRpm(int16_t target) {
  return (int16_t)(target * 0.9f);   // 목표의 90%를 추종 중인 것처럼
}

void replyTelemetry(uint32_t seq, int16_t actual_rpm, uint8_t status) {
  twai_message_t m;
  m.flags = 0;
  m.identifier = ID_TELEM;
  m.data_length_code = 8;

  m.data[0] = seq        & 0xFF;
  m.data[1] = (seq >> 8) & 0xFF;
  m.data[2] = (seq >> 16) & 0xFF;
  m.data[3] = (seq >> 24) & 0xFF;
  m.data[4] = actual_rpm        & 0xFF;
  m.data[5] = (actual_rpm >> 8) & 0xFF;
  m.data[6] = status;
  m.data[7] = xorSum(m.data, 7);

  if (twai_transmit(&m, 0) == ESP_OK) stat_sent++;
  else                                stat_tx_fail++;
}

void handleCommands() {
  twai_message_t m;
  // 큐를 전부 비웁니다. 남겨두면 응답 지연이 누적됩니다.
  while (twai_receive(&m, 0) == ESP_OK) {
    if (m.identifier != ID_CMD || m.data_length_code != 8) continue;

    if (xorSum(m.data, 7) != m.data[7]) { stat_bad_sum++; continue; }

    uint32_t seq = (uint32_t)m.data[0]        | ((uint32_t)m.data[1] << 8)
                 | ((uint32_t)m.data[2] << 16) | ((uint32_t)m.data[3] << 24);

    // seq가 1씩 안 늘면 그 사이 프레임이 유실된 것입니다.
    // 메인의 lost 통계는 왕복 기준이라, 어느 방향에서 빠졌는지는 이 값으로 갈립니다.
    if (seq_seen && seq > last_seq + 1) stat_seq_gap += (seq - last_seq - 1);
    last_seq = seq;
    seq_seen = true;

    last_target_rpm = (int16_t)((uint16_t)m.data[4] | ((uint16_t)m.data[5] << 8));
    last_enable     = m.data[6];
    stat_recv++;

    // 받은 즉시 응답. 왕복 지연에 이쪽 처리 시간이 최소로 들어가게 합니다.
    replyTelemetry(seq, makeActualRpm(last_target_rpm), last_enable ? ST_READY : ST_IDLE);
  }
}

void printStats() {
  twai_status_info_t st;
  twai_get_status_info(&st);

  Serial.print(F("recv "));   Serial.print(stat_recv);
  Serial.print(F("  sent ")); Serial.print(stat_sent);
  Serial.print(F("  seqgap ")); Serial.print(stat_seq_gap);
  Serial.print(F("   last cmd: rpm ")); Serial.print(last_target_rpm);
  Serial.print(F(" en "));              Serial.print(last_enable);
  Serial.print(F("   bus ")); Serial.print(stateName(st.state));
  Serial.print(F(" txerr ")); Serial.print(st.tx_error_counter);
  Serial.print(F(" rxerr ")); Serial.print(st.rx_error_counter);
  if (stat_tx_fail) { Serial.print(F("  txfail ")); Serial.print(stat_tx_fail); }
  if (stat_bad_sum) { Serial.print(F("  badsum ")); Serial.print(stat_bad_sum); }
  Serial.println();

  if (st.state == TWAI_STATE_BUS_OFF) {
    Serial.println(F("  !! BUS_OFF - check CANH/CANL, common GND, 120ohm termination"));
    twai_initiate_recovery();
  }

  stat_recv = stat_sent = stat_bad_sum = stat_tx_fail = stat_seq_gap = 0;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("=========================================="));
  Serial.println(F(" CAN test - SUB (ESP32 FOC board)"));
  Serial.print  (F(" TX=GPIO")); Serial.print(PIN_CAN_TX);
  Serial.print  (F("  RX=GPIO")); Serial.println(PIN_CAN_RX);
  Serial.println(F(" 500 kbit/s, listens 0x100, replies 0x200"));
  Serial.println(F(" Motor is NOT touched by this sketch."));
  Serial.println(F("=========================================="));

  if (!canInit()) { while (1) delay(100); }

  Serial.println(F("CAN started. Waiting for commands from MAIN..."));
  Serial.println(F("If 'recv' stays 0: check common GND and 120ohm termination first."));

  last_stat_ms = millis();
}

void loop() {
  handleCommands();

  unsigned long now = millis();
  if (now - last_stat_ms >= STAT_PERIOD_MS) {
    last_stat_ms = now;
    printStats();
  }
}
