// CAN 통신 테스트 - 메인 컨트롤 보드 (ESP32-S3)
//
// 역할: 마스터. 제어 명령을 주기적으로 송신하고, 서브 보드가 되돌려 보내는
//       텔레메트리를 받아 왕복 지연/유실률/버스 상태를 통계로 출력합니다.
//
// 짝이 되는 스케치: 04_can_test_sub  (ESP32 FOC 보드에 업로드)
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
//   [필수] 두 보드의 GND를 반드시 연결할 것. CANH/CANL만 이으면 동작하지 않습니다.
//   [필수] 버스 양 끝에 120Ω 종단저항. 2노드 구성이면 양쪽 보드에 하나씩입니다.
//          대부분의 SN65HVD230 브레이크아웃 모듈에는 120Ω이 이미 실장되어 있으니
//          모듈을 두 개 쓰면 그대로 맞습니다. 중복 실장 시 60Ω이 되어 통신이 깨집니다.
//
// ─────────────────────────────────────────────────────────────────────
// 프로토콜 (8바이트 고정)
// ─────────────────────────────────────────────────────────────────────
//   0x100  MAIN -> SUB   제어 명령
//     [0..3] uint32  seq        시퀀스 번호
//     [4..5] int16   target_rpm 목표 속도
//     [6]    uint8   enable     모터 활성 플래그
//     [7]    uint8   checksum   0..6 바이트 XOR
//
//   0x200  SUB -> MAIN   텔레메트리
//     [0..3] uint32  seq        받은 명령의 seq 그대로 반향
//     [4..5] int16   actual_rpm 실측 속도 (이 테스트에서는 모의값)
//     [6]    uint8   status     서브 상태 코드
//     [7]    uint8   checksum   0..6 바이트 XOR
//
// seq를 반향시키는 이유: 왕복 지연을 프레임 단위로 정확히 잴 수 있고,
// 어떤 명령이 유실됐는지도 특정됩니다.
//
// 시리얼 115200. 1초마다 통계 출력.

#include "driver/twai.h"

// ===== 핀 (ESP32-S3, README 핀 배정표 기준) =====
#define PIN_CAN_TX 1
#define PIN_CAN_RX 2

// ===== CAN ID =====
#define ID_CMD   0x100   // MAIN -> SUB
#define ID_TELEM 0x200   // SUB  -> MAIN

// ===== 송신 주기 =====
// 밸런싱 루프가 요구하는 200Hz를 그대로 걸어 실사용 조건에서 시험합니다.
// 500kbit/s에서 8바이트 프레임은 약 270us. 왕복 2프레임이면 버스 점유율 약 11%.
const unsigned long SEND_PERIOD_US = 5000;   // 5ms = 200Hz
const unsigned long STAT_PERIOD_MS = 1000;

// ===== 왕복 지연 측정용 링버퍼 =====
#define LAT_SLOTS 64
uint32_t send_us[LAT_SLOTS];

// ===== 통계 =====
uint32_t seq_counter = 0;
uint32_t stat_sent = 0, stat_recv = 0, stat_bad_sum = 0, stat_tx_fail = 0;
uint32_t lat_min = 0xFFFFFFFF, lat_max = 0, lat_count = 0;
uint64_t lat_sum = 0;

unsigned long last_send_us = 0;
unsigned long last_stat_ms = 0;

// 테스트용 목표값. 실제 제어 대신 사인파 형태로 흔들어 값이 제대로 전달되는지 봅니다.
int16_t makeTargetRpm() {
  return (int16_t)(1000.0f * sinf(millis() / 2000.0f));
}

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

void sendCommand() {
  twai_message_t m;
  m.flags = 0;                 // extd/rtr 등 전부 해제 = 표준 데이터 프레임
  m.identifier = ID_CMD;
  m.data_length_code = 8;

  uint32_t seq = seq_counter++;
  int16_t  rpm = makeTargetRpm();

  m.data[0] = seq        & 0xFF;
  m.data[1] = (seq >> 8) & 0xFF;
  m.data[2] = (seq >> 16) & 0xFF;
  m.data[3] = (seq >> 24) & 0xFF;
  m.data[4] = rpm        & 0xFF;
  m.data[5] = (rpm >> 8) & 0xFF;
  m.data[6] = 1;               // enable
  m.data[7] = xorSum(m.data, 7);

  // 타임아웃 0: 큐가 차 있으면 기다리지 않고 실패 처리합니다.
  // 제어 루프를 막지 않기 위해서입니다.
  if (twai_transmit(&m, 0) == ESP_OK) {
    send_us[seq % LAT_SLOTS] = micros();
    stat_sent++;
  } else {
    stat_tx_fail++;
  }
}

void receiveTelemetry() {
  twai_message_t m;
  // 큐에 쌓인 것을 모두 비웁니다. 남겨두면 지연이 누적됩니다.
  while (twai_receive(&m, 0) == ESP_OK) {
    if (m.identifier != ID_TELEM || m.data_length_code != 8) continue;

    if (xorSum(m.data, 7) != m.data[7]) { stat_bad_sum++; continue; }

    uint32_t seq = (uint32_t)m.data[0]        | ((uint32_t)m.data[1] << 8)
                 | ((uint32_t)m.data[2] << 16) | ((uint32_t)m.data[3] << 24);

    uint32_t dt = micros() - send_us[seq % LAT_SLOTS];
    if (dt < 1000000UL) {           // 1초 넘는 값은 슬롯 재사용으로 인한 오측정
      if (dt < lat_min) lat_min = dt;
      if (dt > lat_max) lat_max = dt;
      lat_sum += dt;
      lat_count++;
    }
    stat_recv++;
  }
}

void printStats() {
  twai_status_info_t st;
  twai_get_status_info(&st);

  uint32_t lost = (stat_sent > stat_recv) ? (stat_sent - stat_recv) : 0;
  float loss_pct = stat_sent ? (100.0f * lost / stat_sent) : 0.0f;

  Serial.print(F("sent "));   Serial.print(stat_sent);
  Serial.print(F("  recv ")); Serial.print(stat_recv);
  Serial.print(F("  lost ")); Serial.print(lost);
  Serial.print(F(" ("));      Serial.print(loss_pct, 1); Serial.print(F("%)"));

  if (lat_count) {
    Serial.print(F("   RTT us min/avg/max "));
    Serial.print(lat_min);                        Serial.print('/');
    Serial.print((uint32_t)(lat_sum / lat_count)); Serial.print('/');
    Serial.print(lat_max);
  } else {
    Serial.print(F("   RTT --"));
  }

  Serial.print(F("   bus ")); Serial.print(stateName(st.state));
  Serial.print(F(" txerr ")); Serial.print(st.tx_error_counter);
  Serial.print(F(" rxerr ")); Serial.print(st.rx_error_counter);
  if (stat_tx_fail) { Serial.print(F("  txfail ")); Serial.print(stat_tx_fail); }
  if (stat_bad_sum) { Serial.print(F("  badsum ")); Serial.print(stat_bad_sum); }
  Serial.println();

  // 버스오프는 배선 불량의 대표 증상입니다. 자동 복구를 시도합니다.
  if (st.state == TWAI_STATE_BUS_OFF) {
    Serial.println(F("  !! BUS_OFF - check CANH/CANL, common GND, 120ohm termination"));
    twai_initiate_recovery();
  }

  // 창 단위 통계로 리셋
  stat_sent = stat_recv = stat_bad_sum = stat_tx_fail = 0;
  lat_min = 0xFFFFFFFF; lat_max = 0; lat_sum = 0; lat_count = 0;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("=========================================="));
  Serial.println(F(" CAN test - MAIN (ESP32-S3)"));
  Serial.print  (F(" TX=GPIO")); Serial.print(PIN_CAN_TX);
  Serial.print  (F("  RX=GPIO")); Serial.println(PIN_CAN_RX);
  Serial.println(F(" 500 kbit/s, cmd 0x100 @200Hz, telem 0x200"));
  Serial.println(F("=========================================="));

  if (!canInit()) { while (1) delay(100); }

  Serial.println(F("CAN started. Waiting for the SUB board..."));
  Serial.println(F("If 'recv' stays 0: check common GND and 120ohm termination first."));

  last_send_us = micros();
  last_stat_ms = millis();
}

void loop() {
  unsigned long now_us = micros();
  if (now_us - last_send_us >= SEND_PERIOD_US) {
    last_send_us += SEND_PERIOD_US;
    sendCommand();
  }

  receiveTelemetry();

  unsigned long now_ms = millis();
  if (now_ms - last_stat_ms >= STAT_PERIOD_MS) {
    last_stat_ms = now_ms;
    printStats();
  }
}
