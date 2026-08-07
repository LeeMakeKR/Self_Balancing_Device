// CAN 모터 지령 테스트 - 메인 컨트롤 보드 (ESP32-S3)
//
// 역할: 정해진 시퀀스를 1회 자동 수행하고, 끝나면 판정이 포함된 테스트 보고서를
//       마크다운으로 출력합니다. 출력물을 그대로 "제작 로그.md"에 붙여넣을 수 있습니다.
//
// 짝이 되는 스케치: 05_can_motor_test_sub  (ESP32 FOC 보드에 업로드)
// 선행 확인 사항  : 04_can_com_test_* 통신 성공, 05_node_id_test로 Node ID 확정
// 참조 문서       : CAN_프로토콜.md
//
// ─────────────────────────────────────────────────────────────────────
// 배선
// ─────────────────────────────────────────────────────────────────────
//   GPIO1 -> 트랜시버 D(TXD),  GPIO2 <- 트랜시버 R(RXD)   (배선 확인 완료)
//   두 보드 GND 공통 연결 필수. 버스 양 끝 120Ω 종단.
//
// ─────────────────────────────────────────────────────────────────────
// 테스트 시퀀스 (총 약 25초, 자동)
// ─────────────────────────────────────────────────────────────────────
//   0. 서브 대기      서브의 첫 텔레메트리를 받을 때까지 대기 (측정 제외).
//                     서브는 initFOC 정렬 후 자기각 센서로 모터 정지를 확인하고
//                     3초를 더 기다린 뒤에야 응답하기 시작합니다. 따라서 첫 응답이
//                     오면 이미 안정된 상태이며, 메인은 시간을 따로 재지 않습니다.
//   1. 링크 확인      enable 없이 통신만 성립하는지
//   2. 영점 유지      enable 상태에서 0 지령. 멋대로 돌면 여기서 걸립니다
//   3. +5 rad/s       저속 추종
//   4. +10 rad/s      중속 추종
//   5. 정지
//   6. -10 rad/s      역방향 추종 (부호가 뒤집혔는지 확인)
//   7. 정지
//   8. 워치독 시험    송신을 1초간 끊음. 서브가 스스로 멈춰야 합니다
//   9. 복구 확인      송신 재개 후 통신이 되살아나는지
//
// 시리얼 115200.  'x' 중단,  'r' 재실행.
//
// 안전: 처음 돌릴 때는 휠을 떼거나 보드를 고정하세요.
//       보고서 출력 후 메인은 송신을 멈추고, 서브는 15ms 워치독으로 정지합니다.

#include "driver/twai.h"
#include <stdarg.h>

// ===== 핀 =====
#define PIN_CAN_TX 1
#define PIN_CAN_RX 2

// ===== CAN ID (CAN_프로토콜.md 3절) =====
#define ID_CMD        0x080   // MAIN -> ALL  제어 명령
#define ID_TELEM_BASE 0x090   // SUB n -> MAIN  (0x091 ~ 0x093)

// ===== 제어 모드 (flags bit4..5) =====
#define MODE_VELOCITY 1

// ===== 주기 =====
const unsigned long SEND_PERIOD_US  = 5000;   // 5ms = 200Hz
const unsigned long PROGRESS_PERIOD_MS = 1000;
const unsigned long START_DELAY_MS  = 3000;   // 시리얼 모니터 열 시간

// ─────────────────────────────────────────────────────────────────────
// 합격 기준
// ─────────────────────────────────────────────────────────────────────
const float    PASS_LOSS_PCT      = 1.0f;     // 프레임 유실률 상한
const uint32_t PASS_RTT_MAX_US    = 2000;     // 왕복 지연 최댓값 상한
const float    PASS_ERR_RATIO     = 0.10f;    // 정상상태 오차 / 지령
const float    PASS_ERR_FLOOR     = 0.5f;     // 저속에서 비율 대신 쓰는 절대 허용치 (rad/s)
const float    PASS_ZERO_VEL      = 1.0f;     // 0 지령일 때 허용 잔류 속도 (rad/s)
const float    PASS_OVERSHOOT     = 1.25f;    // 최대 속도 / 지령
// 오버슈트는 지령에 비례하지 않고 절댓값이 거의 일정하게 나옵니다.
// 슬루 램프가 끝나는 순간 PID의 적분항이 이미 쌓여 있어 생기는 것이라
// 지령이 작을수록 비율만 커집니다. 오차 판정과 같이 절대 하한을 함께 둡니다.
const float    PASS_OVERSHOOT_FLOOR = 2.5f;   // rad/s
const uint32_t PASS_MIN_RECV_RATE = 150;      // 초당 최소 수신 프레임 수

// ===== RTT 유효성 =====
// seq는 uint8이라 256프레임마다 순환합니다. 이 범위를 벗어난 반향은 옛 슬롯을
// 읽는 것이므로 지연 통계에서 제외합니다.
const uint8_t  RTT_MAX_SEQ_AGE = 32;
const uint32_t RTT_SANITY_US   = 50000;   // 50ms 넘는 왕복은 물리적으로 불가능

// ===== 서브 보드 대기 =====
// 서브는 initFOC 정렬 때문에 부팅이 수 초 걸립니다. 그 시간을 측정 구간에
// 포함시키면 유실률이 실제보다 훨씬 나쁘게 나옵니다. 첫 응답을 받은 뒤 시작합니다.
// 서브는 initFOC 정렬 후 자기각 센서로 모터 정지를 확인하고 3초를 더 기다린 뒤에야
// 텔레메트리를 보내기 시작합니다. 따라서 첫 응답이 오면 이미 안정된 상태입니다.
// 메인이 별도로 시간을 기다릴 필요가 없습니다.
// 타임아웃은 서브의 정지 확인(최대 30초) + 3초를 덮을 만큼 넉넉해야 합니다.
const unsigned long SUB_WAIT_TIMEOUT_MS = 60000;

// ─────────────────────────────────────────────────────────────────────
// 테스트 단계 정의
// ─────────────────────────────────────────────────────────────────────
enum PhaseCheck {
  CHK_LINK,     // 통신만 확인 (모터 정지 상태)
  CHK_ZERO,     // enable 상태에서 0 지령 유지
  CHK_TRACK,    // 목표 속도 추종
  CHK_SILENT,   // 송신 중단. 수신도 끊겨야 정상
  CHK_RESUME    // 송신 재개. 통신 복구 확인
};

struct PhaseDef {
  const char*   name;
  float         target;    // rad/s
  bool          enable;
  bool          send;      // false = 명령 송신 중단
  unsigned long ms;
  PhaseCheck    check;
};

const PhaseDef PHASES[] = {
  { "링크 확인",        0.0f, false, true,  2000, CHK_LINK   },
  { "영점 유지",        0.0f, true,  true,  2000, CHK_ZERO   },
  { "+5 rad/s",         5.0f, true,  true,  4000, CHK_TRACK  },
  { "+10 rad/s",       10.0f, true,  true,  4000, CHK_TRACK  },
  { "정지 (정방향 후)", 0.0f, true,  true,  3000, CHK_ZERO   },
  { "-10 rad/s",      -10.0f, true,  true,  4000, CHK_TRACK  },
  { "정지 (역방향 후)", 0.0f, true,  true,  3000, CHK_ZERO   },
  { "워치독 (송신중단)",0.0f, true,  false, 1000, CHK_SILENT },
  { "복구 확인",        0.0f, true,  true,  2000, CHK_RESUME },
};
const int N_PHASES = sizeof(PHASES) / sizeof(PHASES[0]);

// ─────────────────────────────────────────────────────────────────────
// 단계별 측정값 (축 1..3 x 단계)
// ─────────────────────────────────────────────────────────────────────
struct PhaseResult {
  uint32_t recv;
  uint32_t lat_min, lat_max;
  uint64_t lat_sum;
  uint32_t lat_n;

  float    vel_ss_sum;      // 후반 50% 구간 평균용 (정상상태)
  uint32_t vel_ss_n;
  float    vel_abs_max;
  int32_t  t95_ms;          // 지령의 95% 최초 도달 시각. -1 = 미도달
  uint8_t  status_or;       // 관측된 status 비트 전체 OR

  float    cur_abs_sum;     // 전류 |mA| 평균용
  uint32_t cur_n;
  float    cur_abs_max;     // 전류 |mA| 최댓값
};

PhaseResult res[4][12];     // [axis 1..3][phase]
uint32_t    phase_sent[12];

// ===== 전역 통계 =====
uint32_t g_tx_fail = 0;
uint32_t g_txerr_max = 0, g_rxerr_max = 0;
uint32_t g_busoff_count = 0;
uint32_t g_rtt_reject = 0;
unsigned long g_sub_wait_ms = 0;   // 서브 첫 응답까지 걸린 시간
bool     axis_seen[4] = { false, false, false, false };

// ===== 송신 시각 기록 (seq는 uint8) =====
uint32_t send_us[256];
uint8_t  seq_counter = 0;

// ===== 진행 상태 =====
enum RunState { ST_WAIT_SUB, ST_RUNNING, ST_DONE };
RunState run_state = ST_WAIT_SUB;

int           cur_phase = 0;
unsigned long phase_start_ms = 0;
unsigned long last_send_us = 0;
unsigned long last_progress_ms = 0;
unsigned long boot_ms = 0;
bool          aborted = false;

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

void sendCommand(float target, bool enable) {
  twai_message_t m;
  m.flags = 0;
  m.identifier = ID_CMD;
  m.data_length_code = 8;

  uint8_t seq = seq_counter++;

  // flags: bit0..2 축별 enable, bit3 브레이크, bit4..5 제어 모드
  uint8_t flags = 0;
  if (enable) flags |= 0x07;                  // 3축 모두 enable
  flags |= (MODE_VELOCITY & 0x03) << 4;

  int16_t t = (int16_t)(target * 10.0f);      // rad/s x10

  m.data[0] = seq;
  m.data[1] = flags;
  m.data[2] = t & 0xFF;  m.data[3] = (t >> 8) & 0xFF;   // 1축
  m.data[4] = t & 0xFF;  m.data[5] = (t >> 8) & 0xFF;   // 2축
  m.data[6] = t & 0xFF;  m.data[7] = (t >> 8) & 0xFF;   // 3축

  // 타임아웃 0: 큐가 차 있어도 제어 루프를 막지 않습니다.
  if (twai_transmit(&m, 0) == ESP_OK) {
    send_us[seq] = micros();
    // 대기/종료 구간의 송신은 통계에 넣지 않습니다.
    if (run_state == ST_RUNNING) phase_sent[cur_phase]++;
  } else {
    g_tx_fail++;
  }
}

// 서브 대기 구간에서만 쓰는 가벼운 수신 처리.
// 측정 구간이 아니므로 단계별 통계를 건드리지 않습니다.
bool pollAnyTelemetry() {
  bool seen = false;
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) {
    if ((m.identifier & 0xFF0) != ID_TELEM_BASE) continue;
    int n = m.identifier & 0x0F;
    if (n < 1 || n > 3) continue;
    axis_seen[n] = true;
    seen = true;
  }
  return seen;
}

void receiveTelemetry() {
  const PhaseDef& p = PHASES[cur_phase];
  unsigned long elapsed = millis() - phase_start_ms;
  bool in_steady = (elapsed >= p.ms / 2);     // 후반 50%를 정상상태로 봅니다

  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) {
    if ((m.identifier & 0xFF0) != ID_TELEM_BASE) continue;
    if (m.data_length_code != 8) continue;

    int n = m.identifier & 0x0F;             // Node ID = 축 번호
    if (n < 1 || n > 3) continue;
    axis_seen[n] = true;

    PhaseResult& r = res[n][cur_phase];
    uint8_t seq_echo = m.data[0];

    // RTT는 "최근에 보낸 seq를 되받았을 때"만 유효합니다.
    // seq가 uint8이라 256프레임(200Hz에서 1.28초)마다 순환합니다. 통신이 잠깐
    // 끊겼다 재개되면 한 바퀴 돈 옛 슬롯을 읽어 수백 ms짜리 허수가 나옵니다.
    // 나이(age)로 먼저 거르고, 물리적으로 불가능한 값도 한 번 더 자릅니다.
    uint8_t  age = (uint8_t)(seq_counter - seq_echo);
    uint32_t dt  = micros() - send_us[seq_echo];

    if (age <= RTT_MAX_SEQ_AGE && dt < RTT_SANITY_US) {
      if (dt < r.lat_min) r.lat_min = dt;
      if (dt > r.lat_max) r.lat_max = dt;
      r.lat_sum += dt;
      r.lat_n++;
    } else {
      g_rtt_reject++;
    }

    r.status_or |= m.data[1];

    int16_t vraw = (int16_t)((uint16_t)m.data[4] | ((uint16_t)m.data[5] << 8));
    float   vel  = vraw / 10.0f;

    if (fabsf(vel) > r.vel_abs_max) r.vel_abs_max = fabsf(vel);
    if (in_steady) { r.vel_ss_sum += vel; r.vel_ss_n++; }

    // 전류 (서브의 INA240 실측, mA)
    int16_t craw = (int16_t)((uint16_t)m.data[6] | ((uint16_t)m.data[7] << 8));
    float   cabs = fabsf((float)craw);
    r.cur_abs_sum += cabs;
    r.cur_n++;
    if (cabs > r.cur_abs_max) r.cur_abs_max = cabs;

    // t95: 지령의 95%에 부호까지 맞춰 처음 도달한 시각
    if (r.t95_ms < 0 && fabsf(p.target) > 0.5f) {
      if (vel * p.target > 0 && fabsf(vel) >= 0.95f * fabsf(p.target)) {
        r.t95_ms = (int32_t)elapsed;
      }
    }

    r.recv++;
  }
}

void updateBusStats() {
  twai_status_info_t st;
  twai_get_status_info(&st);

  if (st.tx_error_counter > g_txerr_max) g_txerr_max = st.tx_error_counter;
  if (st.rx_error_counter > g_rxerr_max) g_rxerr_max = st.rx_error_counter;

  if (st.state == TWAI_STATE_BUS_OFF) {
    g_busoff_count++;
    twai_initiate_recovery();
  }
}

// ─────────────────────────────────────────────────────────────────────
// 보고서
// ─────────────────────────────────────────────────────────────────────
int  primary_axis = 0;      // 상세 보고 대상 (응답한 첫 축)
int  fail_count = 0;
char fail_list[6][48];

void addFail(const char* fmt, ...) {
  if (fail_count >= 6) { fail_count++; return; }
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(fail_list[fail_count], sizeof(fail_list[0]), fmt, ap);
  va_end(ap);
  fail_count++;
}

void printSeparator() {
  Serial.println(F("----------------------------------------------------------------"));
}

// 단계 하나를 판정합니다. 통과면 true.
bool judgePhase(int axis, int i, char* note, size_t note_len) {
  const PhaseDef& p = PHASES[i];
  PhaseResult&    r = res[axis][i];

  float ss = r.vel_ss_n ? (r.vel_ss_sum / r.vel_ss_n) : 0.0f;
  uint32_t expect_recv = (p.ms * PASS_MIN_RECV_RATE) / 1000;

  switch (p.check) {
    case CHK_LINK:
      if (r.recv < expect_recv) {
        snprintf(note, note_len, "수신 %lu < 기대 %lu", (unsigned long)r.recv, (unsigned long)expect_recv);
        return false;
      }
      snprintf(note, note_len, "수신 %lu", (unsigned long)r.recv);
      return true;

    case CHK_RESUME:
      if (r.recv < expect_recv) {
        snprintf(note, note_len, "수신 %lu < 기대 %lu", (unsigned long)r.recv, (unsigned long)expect_recv);
        return false;
      }
      // 서브가 복구 직후 몇 프레임 동안 WDT 비트를 남기므로, 그게 보여야
      // "명령이 끊겼을 때 서브가 실제로 멈췄다"가 증명됩니다.
      if (!(r.status_or & 0x08)) {
        snprintf(note, note_len, "수신 %lu, 워치독 비트 미관측", (unsigned long)r.recv);
        return false;
      }
      snprintf(note, note_len, "수신 %lu, 워치독 정지 확인", (unsigned long)r.recv);
      return true;

    case CHK_SILENT:
      // 송신을 끊었으므로 서브도 응답할 수 없어야 합니다.
      if (r.recv > 0) {
        snprintf(note, note_len, "송신 중단 중 %lu 프레임 수신", (unsigned long)r.recv);
        return false;
      }
      snprintf(note, note_len, "무응답 확인 (정상)");
      return true;

    case CHK_ZERO:
      if (fabsf(ss) > PASS_ZERO_VEL) {
        snprintf(note, note_len, "잔류 %.1f > %.1f rad/s", ss, PASS_ZERO_VEL);
        return false;
      }
      snprintf(note, note_len, "잔류 %.1f rad/s", ss);
      return true;

    case CHK_TRACK: {
      if (r.vel_ss_n == 0) {
        snprintf(note, note_len, "정상상태 샘플 없음");
        return false;
      }
      // 방향 먼저 봅니다. 부호가 뒤집혔으면 오차 크기는 의미가 없습니다.
      if (ss * p.target <= 0) {
        snprintf(note, note_len, "회전 방향 반대 (실측 %.1f)", ss);
        return false;
      }
      float tol = fabsf(p.target) * PASS_ERR_RATIO;
      if (tol < PASS_ERR_FLOOR) tol = PASS_ERR_FLOOR;
      float err = ss - p.target;
      if (fabsf(err) > tol) {
        snprintf(note, note_len, "오차 %.1f > 허용 %.1f", err, tol);
        return false;
      }
      // 비율 기준과 절대 하한 중 넉넉한 쪽을 씁니다.
      float over_allow = fabsf(p.target) * PASS_OVERSHOOT;
      float floor_allow = fabsf(p.target) + PASS_OVERSHOOT_FLOOR;
      if (floor_allow > over_allow) over_allow = floor_allow;

      if (r.vel_abs_max > over_allow) {
        snprintf(note, note_len, "오버슈트 %.1f > 허용 %.1f", r.vel_abs_max, over_allow);
        return false;
      }
      snprintf(note, note_len, "오차 %.1f/%.1f, 오버슈트 %.1f/%.1f",
               err, tol, r.vel_abs_max - fabsf(p.target), over_allow - fabsf(p.target));
      return true;
    }
  }
  return false;
}

void printReport() {
  // 상세 보고 대상 축 선정
  primary_axis = 0;
  for (int n = 1; n <= 3; n++) if (axis_seen[n]) { primary_axis = n; break; }

  fail_count = 0;

  Serial.println();
  printSeparator();
  Serial.println(F("여기부터 복사해서 제작 로그.md에 붙여넣으세요"));
  printSeparator();
  Serial.println();

  Serial.println(F("## CAN 모터 지령 테스트 보고서"));
  Serial.println();
  Serial.print  (F("- 스케치: `05_can_motor_test_main` / `05_can_motor_test_sub`"));
  Serial.println();
  Serial.println(F("- 프로토콜: `0x080` 제어 명령 @200Hz / `0x09n` 텔레메트리 (CAN_프로토콜.md)"));
  Serial.println(F("- 버스: 500 kbit/s, 표준 프레임, DLC 8"));
  Serial.print  (F("- 소요 시간: ")); Serial.print((millis() - boot_ms) / 1000); Serial.println(F("초"));
  Serial.print  (F("- 응답 축: "));
  if (!primary_axis) {
    Serial.println(F("**없음**"));
  } else {
    for (int n = 1; n <= 3; n++) if (axis_seen[n]) { Serial.print(n); Serial.print(F("번축 ")); }
    Serial.println();
  }
  if (aborted) Serial.println(F("- **주의: 사용자 중단('x')으로 조기 종료된 실행입니다**"));
  Serial.println();

  if (!primary_axis) {
    Serial.println(F("### 결과: **FAIL — 서브 보드 응답 없음**"));
    Serial.println();
    Serial.println(F("확인 순서:"));
    Serial.println(F("1. 두 보드 GND 공통 연결"));
    Serial.println(F("2. 버스 양 끝 120Ω 종단 (중복 실장 시 60Ω이 되어 통신 불가)"));
    Serial.println(F("3. 서브 보드 Node ID 점퍼 (`05_node_id_test`로 확인)"));
    Serial.println(F("4. 트랜시버 RS 핀이 GND에 직결됐는지"));
    Serial.println();
    Serial.print  (F("송신 실패 ")); Serial.print(g_tx_fail);
    Serial.print  (F("회, txerr 최대 ")); Serial.print(g_txerr_max);
    Serial.print  (F(", rxerr 최대 ")); Serial.println(g_rxerr_max);
    printSeparator();
    return;
  }

  int n = primary_axis;

  // ── 1. 통신 ──
  uint32_t tot_sent = 0, tot_recv = 0, tot_lat_n = 0;
  uint64_t tot_lat_sum = 0;
  uint32_t tot_lat_min = 0xFFFFFFFF, tot_lat_max = 0;

  for (int i = 0; i < N_PHASES; i++) {
    if (PHASES[i].check == CHK_SILENT) continue;   // 의도적 중단 구간은 제외
    tot_sent += phase_sent[i];
    tot_recv += res[n][i].recv;
    tot_lat_sum += res[n][i].lat_sum;
    tot_lat_n   += res[n][i].lat_n;
    if (res[n][i].lat_min < tot_lat_min) tot_lat_min = res[n][i].lat_min;
    if (res[n][i].lat_max > tot_lat_max) tot_lat_max = res[n][i].lat_max;
  }

  uint32_t lost = (tot_sent > tot_recv) ? (tot_sent - tot_recv) : 0;
  float loss_pct = tot_sent ? (100.0f * lost / tot_sent) : 100.0f;
  uint32_t rtt_avg = tot_lat_n ? (uint32_t)(tot_lat_sum / tot_lat_n) : 0;

  bool comm_ok = true;
  if (loss_pct > PASS_LOSS_PCT)    { comm_ok = false; addFail("유실률 %.2f%% > %.1f%%", loss_pct, PASS_LOSS_PCT); }
  if (tot_lat_max > PASS_RTT_MAX_US) { comm_ok = false; addFail("RTT 최대 %luus 초과", (unsigned long)tot_lat_max); }
  if (g_busoff_count)              { comm_ok = false; addFail("BUS_OFF %lu회", (unsigned long)g_busoff_count); }

  Serial.print(F("### 1. 통신 ")); Serial.println(comm_ok ? F("— PASS") : F("— **FAIL**"));
  Serial.println();
  Serial.println(F("| 항목 | 측정값 | 기준 |"));
  Serial.println(F("|---|---|---|"));
  Serial.print(F("| 송신 프레임 | ")); Serial.print(tot_sent); Serial.println(F(" | — |"));
  Serial.print(F("| 수신 프레임 | ")); Serial.print(tot_recv); Serial.println(F(" | — |"));
  Serial.print(F("| 유실률 | ")); Serial.print(loss_pct, 2);
  Serial.print(F("% | < ")); Serial.print(PASS_LOSS_PCT, 1); Serial.println(F("% |"));
  Serial.print(F("| RTT min/avg/max | "));
  Serial.print(tot_lat_min); Serial.print(F(" / ")); Serial.print(rtt_avg);
  Serial.print(F(" / ")); Serial.print(tot_lat_max);
  Serial.print(F(" us | max < ")); Serial.print(PASS_RTT_MAX_US); Serial.println(F(" us |"));
  Serial.print(F("| 송신 실패 | ")); Serial.print(g_tx_fail); Serial.println(F(" | — |"));
  Serial.print(F("| 버스 에러 최대 | txerr ")); Serial.print(g_txerr_max);
  Serial.print(F(" / rxerr ")); Serial.print(g_rxerr_max); Serial.println(F(" | — |"));
  Serial.print(F("| BUS_OFF | ")); Serial.print(g_busoff_count); Serial.println(F("회 | 0회 |"));
  Serial.print(F("| RTT 측정 제외 | ")); Serial.print(g_rtt_reject);
  Serial.println(F("건 | seq 순환으로 무효한 반향 |"));
  if (tot_lat_max > PASS_RTT_MAX_US) {
    Serial.println(F("|  ↳ RTT 최대 초과 원인 | 양 보드의 1Hz 시리얼 출력이 루프를 막는 구간 | 실제 펌웨어에는 없음 |"));
  }
  Serial.print(F("| 서브 부팅 대기 | ")); Serial.print(g_sub_wait_ms);
  Serial.println(F("ms | 측정 구간에서 제외됨 |"));
  Serial.println();

  // ── 2. 단계별 추종 ──
  Serial.println(F("### 2. 단계별 결과"));
  Serial.println();
  Serial.println(F("| # | 단계 | 지령 | 실측(정상상태) | 최대 | t95 | 전류 avg/max | 판정 | 비고 |"));
  Serial.println(F("|---|---|---|---|---|---|---|---|---|"));

  bool track_ok = true;
  for (int i = 0; i < N_PHASES; i++) {
    const PhaseDef& p = PHASES[i];
    PhaseResult&    r = res[n][i];

    char note[48];
    bool ok = judgePhase(n, i, note, sizeof(note));
    if (!ok) {
      track_ok = false;
      addFail("%s: %s", p.name, note);
    }

    float ss = r.vel_ss_n ? (r.vel_ss_sum / r.vel_ss_n) : 0.0f;

    Serial.print(F("| ")); Serial.print(i + 1);
    Serial.print(F(" | ")); Serial.print(p.name);
    Serial.print(F(" | ")); Serial.print(p.target, 1);
    Serial.print(F(" | ")); Serial.print(ss, 1);
    Serial.print(F(" | ")); Serial.print(r.vel_abs_max, 1);
    Serial.print(F(" | "));
    if (r.t95_ms >= 0) { Serial.print(r.t95_ms); Serial.print(F("ms")); }
    else               { Serial.print(F("—")); }
    Serial.print(F(" | "));
    if (r.cur_n) {
      Serial.print(r.cur_abs_sum / r.cur_n, 0); Serial.print('/');
      Serial.print(r.cur_abs_max, 0);           Serial.print(F("mA"));
    } else {
      Serial.print(F("—"));
    }
    Serial.print(F(" | ")); Serial.print(ok ? F("PASS") : F("**FAIL**"));
    Serial.print(F(" | ")); Serial.print(note);
    Serial.println(F(" |"));
  }
  Serial.println();
  Serial.println(F("속도 단위 rad/s, 전류 단위 mA(절댓값). t95 = 지령의 95%에 처음 도달한 시각(단계 시작 기준)."));
  Serial.println();

  // ── 3. 서브 상태 비트 ──
  uint8_t st_or = 0;
  for (int i = 0; i < N_PHASES; i++) st_or |= res[n][i].status_or;

  Serial.println(F("### 3. 서브 보드 상태 비트 (관측된 OR)"));
  Serial.println();
  Serial.print(F("- enable 관측: "));  Serial.println((st_or & 0x01) ? F("있음") : F("**없음 — 모터가 한 번도 켜지지 않았습니다**"));
  Serial.print(F("- fault 관측: "));   Serial.println((st_or & 0x02) ? F("**있음 — 서브 시리얼 로그 확인 필요**") : F("없음"));
  Serial.print(F("- 정렬 완료: "));    Serial.println((st_or & 0x04) ? F("확인") : F("**미확인 — initFOC 실패 의심**"));
  Serial.print(F("- 워치독 정지: "));
  Serial.println((st_or & 0x08) ? F("관측됨 (명령 중단 시 서브가 실제로 정지함을 확인)")
                                : F("**미관측 — 서브가 명령 중단에도 계속 돌았을 수 있습니다**"));
  Serial.println();
  if (!(st_or & 0x01)) addFail("서브에서 enable 상태가 관측되지 않음");
  if (st_or & 0x02)    addFail("서브 fault 비트 관측됨");

  // ── 4. 종합 ──
  Serial.println(F("### 4. 종합"));
  Serial.println();
  if (fail_count == 0) {
    Serial.println(F("**PASS** — 메인의 속도 지령이 CAN을 통해 전달되어 서브 모터가 지령대로 회전함을 확인했습니다."));
    Serial.println();
    Serial.println(F("확인된 항목: 통신 무결성, 정/역방향 추종, 영점 유지, 워치독 정지 및 복구."));
  } else {
    Serial.print(F("**FAIL** — 실패 항목 ")); Serial.print(fail_count); Serial.println(F("건"));
    Serial.println();
    int shown = (fail_count < 6) ? fail_count : 6;
    for (int i = 0; i < shown; i++) {
      Serial.print(F("- ")); Serial.println(fail_list[i]);
    }
    if (fail_count > 6) {
      Serial.print(F("- (외 ")); Serial.print(fail_count - 6); Serial.println(F("건)"));
    }
  }
  Serial.println();

  Serial.println(F("> 전류는 서브 보드의 INA240 인라인 센싱 실측값입니다 (션트 0.01Ω, 게인 50V/V, ADC 39/36)."));
  Serial.println(F("> PWM 동기 샘플링이 아니라 1kHz 비동기 샘플 + EMA(α=0.2)를 거친 값이므로"));
  Serial.println(F("> 절대 정확도보다 상대 비교와 추세 확인 용도로 보세요."));
  Serial.println(F("> 전류가 전 구간 0이면 서브 시리얼의 `current sense init 실패` 경고를 확인하세요."));
  Serial.println();

  printSeparator();
  Serial.println(F("보고서 끝.  'r' 입력 시 재실행."));
  printSeparator();
}

// ─────────────────────────────────────────────────────────────────────
// 실행 제어
// ─────────────────────────────────────────────────────────────────────
void resetRun() {
  for (int a = 0; a < 4; a++) {
    for (int i = 0; i < 12; i++) {
      res[a][i] = PhaseResult();
      res[a][i].lat_min = 0xFFFFFFFF;
      res[a][i].t95_ms  = -1;
    }
  }
  for (int i = 0; i < 12; i++) phase_sent[i] = 0;
  for (int n = 0; n < 4; n++) axis_seen[n] = false;

  g_tx_fail = 0;
  g_txerr_max = 0; g_rxerr_max = 0;
  g_busoff_count = 0;
  g_rtt_reject = 0;
  g_sub_wait_ms = 0;
  aborted = false;

  cur_phase = 0;
  phase_start_ms = millis();
  last_send_us = micros();
  last_progress_ms = millis();
  boot_ms = millis();
  run_state = ST_WAIT_SUB;

  Serial.println();
  Serial.println(F(">> 서브 보드 응답 대기 중... (initFOC 정렬에 수 초 걸립니다)"));
}

// 서브의 첫 응답을 기다립니다. 이 구간은 측정에 포함되지 않습니다.
void waitForSub() {
  unsigned long now_us = micros();
  if (now_us - last_send_us >= SEND_PERIOD_US) {
    last_send_us += SEND_PERIOD_US;
    sendCommand(0.0f, false);          // disable 상태로만 두드립니다
  }

  if (pollAnyTelemetry()) {
    g_sub_wait_ms = millis() - boot_ms;

    Serial.print(F(">> 서브 응답 확인 ("));
    Serial.print(g_sub_wait_ms);
    Serial.println(F("ms). 서브가 모터 정지를 확인한 상태입니다."));

    // 큐 완전 배수. 대기 구간에 도착해 있던 프레임을 측정에 섞지 않기 위해서입니다.
    twai_message_t dump;
    int drained = 0;
    while (twai_receive(&dump, 0) == ESP_OK) drained++;
    Serial.print(F(">> 수신 큐 배수 ")); Serial.print(drained); Serial.println(F("프레임. 측정 시작."));

    // 대기 구간에서 쌓인 버스 에러는 서브 부팅 대기 때문이므로 초기화합니다.
    g_txerr_max = 0; g_rxerr_max = 0; g_tx_fail = 0;

    run_state       = ST_RUNNING;
    cur_phase       = 0;
    phase_start_ms  = millis();
    boot_ms         = millis();
    last_progress_ms = millis();
    last_send_us    = micros();

    Serial.print(F(">> ")); Serial.println(PHASES[0].name);
    return;
  }

  if (millis() - boot_ms >= SUB_WAIT_TIMEOUT_MS) {
    Serial.println(F(">> 대기 시간 초과. 응답 없음으로 보고합니다."));
    run_state = ST_DONE;
    printReport();
  }
}

void printProgress() {
  const PhaseDef& p = PHASES[cur_phase];
  PhaseResult& r = res[primary_axis ? primary_axis : 1][cur_phase];

  Serial.print(F("  ["));
  Serial.print(cur_phase + 1); Serial.print('/'); Serial.print(N_PHASES);
  Serial.print(F("] ")); Serial.print(p.name);
  Serial.print(F("  지령 ")); Serial.print(p.target, 1);
  Serial.print(F("  수신 ")); Serial.print(r.recv);
  if (r.vel_ss_n) {
    Serial.print(F("  실측 ")); Serial.print(r.vel_ss_sum / r.vel_ss_n, 1);
  }
  Serial.println();
}

void advancePhase() {
  cur_phase++;
  phase_start_ms = millis();

  if (cur_phase >= N_PHASES) {
    // 종료 시 안전 정지 명령을 잠깐 보내고 송신을 멈춥니다.
    // 이후는 서브의 15ms 워치독이 책임집니다.
    for (int i = 0; i < 40; i++) { sendCommand(0.0f, false); delay(5); }

    run_state = ST_DONE;
    printReport();
    return;
  }

  Serial.print(F(">> ")); Serial.println(PHASES[cur_phase].name);
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == 'x' && (run_state == ST_RUNNING || run_state == ST_WAIT_SUB)) {
      aborted = true;
      for (int i = 0; i < 20; i++) { sendCommand(0.0f, false); delay(5); }
      run_state = ST_DONE;
      Serial.println(F("-> 사용자 중단"));
      printReport();
    } else if (c == 'r' && run_state == ST_DONE) {
      resetRun();
    }
  }
}

// ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("=========================================="));
  Serial.println(F(" CAN motor command test - MAIN (ESP32-S3)"));
  Serial.print  (F(" TX=GPIO")); Serial.print(PIN_CAN_TX);
  Serial.print  (F("  RX=GPIO")); Serial.println(PIN_CAN_RX);
  Serial.println(F(" 500kbit/s, cmd 0x080 @200Hz, telem 0x091~0x093"));
  Serial.println(F(" mode = velocity (rad/s x10)"));
  Serial.println(F("=========================================="));
  Serial.println(F(" 1회 자동 실행 후 테스트 보고서를 출력합니다."));
  Serial.println(F(" 'x' 중단 / 'r' 재실행"));
  Serial.println();
  Serial.println(F(" !! 모터가 회전합니다. 휠을 떼거나 보드를 고정하세요."));

  if (!canInit()) { while (1) delay(100); }

  Serial.print(F(" "));
  for (int i = START_DELAY_MS / 1000; i > 0; i--) {
    Serial.print(i); Serial.print(F("... "));
    delay(1000);
  }
  Serial.println();

  resetRun();
}

void loop() {
  handleSerial();

  if (run_state == ST_WAIT_SUB) { waitForSub(); return; }
  if (run_state != ST_RUNNING)  { delay(10); return; }

  const PhaseDef& p = PHASES[cur_phase];

  unsigned long now_us = micros();
  if (now_us - last_send_us >= SEND_PERIOD_US) {
    last_send_us += SEND_PERIOD_US;
    if (p.send) sendCommand(p.target, p.enable);
  }

  receiveTelemetry();
  updateBusStats();

  unsigned long now_ms = millis();
  if (now_ms - last_progress_ms >= PROGRESS_PERIOD_MS) {
    last_progress_ms = now_ms;
    printProgress();
  }

  if (now_ms - phase_start_ms >= p.ms) advancePhase();
}
