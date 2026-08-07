// 모터 - 센서 상관관계 테스트 (1축 = X축) - 메인 컨트롤 보드 (ESP32-S3)
//
// 역할: 바닥에 놓인 조립체가 충분히 안정된 것을 자이로로 확인한 뒤,
//       CAN으로 모터를 회전시켜 반작용으로 큐브가 반대로 도는 것을 측정하고,
//       모터 지령과 몸체 각속도 사이의 관계를 수치로 뽑아냅니다.
//
// 축 정의: 현재 장착된 모터와 IMU 2개가 1축이며, 이를 X축으로 정의합니다.
//          다만 IMU 칩 내부의 X축이 실제 모터 회전축과 같다는 보장은 없으므로,
//          이 코드는 세 축 모두에 대해 상관을 계산하고 "어느 축이 모터 축인지"를
//          측정으로 판정합니다.
//
// 참조 문서: README.md 핀 배정표,  CAN_프로토콜.md
//
// ─────────────────────────────────────────────────────────────────────
// [안전] 처음 돌릴 때 반드시 읽을 것
// ─────────────────────────────────────────────────────────────────────
//   - 이 테스트는 모터를 실제로 돌립니다. 반작용 휠이 장착된 상태로 돌리면
//     조립체가 바닥에서 실제로 회전합니다. 주변을 비우세요.
//   - 기본 목표는 ±150 rad/s(약 1430 RPM)입니다. 실측 최고 회전의 절반 수준이며
//     조립체가 바닥에서 눈에 띄게 회전합니다. 불안하면 - 키로 낮춰 시작하세요.
//   - + / - 키로 목표를 실시간 조정할 수 있습니다. 상한은 250 rad/s입니다.
//   - x 를 누르면 즉시 E-STOP(0x010)을 보내고 정지합니다.
//   - 보고서 출력 후 메인은 0 지령을 유지하고 모터를 disable 합니다.
//   - 서브 보드는 0x080을 15ms 이상 못 받으면 스스로 출력을 차단합니다.
//
// ─────────────────────────────────────────────────────────────────────
// 측정 원리
// ─────────────────────────────────────────────────────────────────────
//   반작용 휠은 각운동량 보존으로 동작합니다. 외부 토크가 없다면
//
//       I_body * w_body + I_wheel * w_wheel = 일정
//
//   이므로, 휠 속도를 dw 만큼 올리면 몸체는 반대로
//
//       w_body = -(I_wheel / I_body) * dw
//
//   만큼 돕니다. 즉 몸체 각속도와 휠 속도는 음의 비례 관계이고,
//   그 기울기의 크기가 곧 관성비 I_wheel / I_body 입니다.
//
//   이 관계는 "각운동량이 보존되는 동안"만 성립합니다. 바닥에 놓여 있으면
//   마찰이 몸체 운동량을 계속 빼앗아 가므로, 휠이 일정 속도로 도는 구간에서는
//   몸체가 서서히 멈춥니다. 그래서 이 코드는 휠이 실제로 가속/감속하는 동안의
//   표본만 상관 계산에 씁니다. 그 구간이 모델이 성립하는 구간입니다.
//
//   반작용 토크가 나오는 것도 가속 구간뿐입니다. 등속이 되면 토크가 0이 되고
//   바닥 마찰만 남습니다. 따라서 반작용을 크게 만들려면
//     - 목표 속도를 높여 가속 구간을 길게 끌고
//     - 그 목표에 도달할 만큼 펄스 시간을 충분히 준다
//   이 두 가지가 함께 필요합니다. 실측 최대 각가속도가 약 138 rad/s²이므로
//   150 rad/s에 도달하는 데만 약 1.09초가 걸립니다.
//
// ─────────────────────────────────────────────────────────────────────
// 시험 절차 (자동 진행)
// ─────────────────────────────────────────────────────────────────────
//   1) 서브 보드 대기      첫 텔레메트리가 올 때까지
//   2) 바닥 안정화 대기    자이로 크기가 문턱 아래로 계속 유지될 때까지
//   3) 바이어스 측정       안정된 상태에서 자이로 영점을 잽니다
//   4) 펄스 시퀀스         +지령 / 정지 / -지령 / 정지 를 반복
//   5) 보고서 자동 출력    후 모터 정지
//
// ─────────────────────────────────────────────────────────────────────
// 출력하는 변수
// ─────────────────────────────────────────────────────────────────────
//   축별로
//     k        : 최소자승 기울기. w_body = k * w_wheel 의 k.
//                음수여야 정상입니다(반작용). |k| 가 관성비 I_wheel/I_body.
//     r        : 피어슨 상관계수. 반작용 모델이 얼마나 잘 맞는지.
//                모터 축에서만 1에 가깝고 나머지 축은 0 근처여야 합니다.
//     resid    : k 보정 후 남는 잔차 RMS (dps).
//   전체
//     모터 축   : |r| 이 가장 큰 축. 이 축이 실제 모터 회전축입니다.
//     반작용 부호 : 모터 + 지령일 때 몸체가 어느 쪽으로 도는지.
//     반응 지연 : 지령 변화 시점부터 몸체가 반응하기 시작할 때까지 (ms).
//     펄스별 재현성 : 같은 지령에 대해 매번 같은 응답이 나오는지.
//
// 시리얼 115200.
//   x : 즉시 정지 (E-STOP 송신)
//   r : 테스트 재시작
//   h : 도움말

#include <SPI.h>
#include "driver/twai.h"

// ─────────────────────────────────────────────────────────────────────
// 핀
// ─────────────────────────────────────────────────────────────────────
// IMU SPI (README 핀 배정표)
#define PIN_SPI_MISO 4
#define PIN_SPI_MOSI 5
#define PIN_SPI_SCLK 6
#define PIN_CS_1     7
#define PIN_CS_2     15

// CAN (CAN_프로토콜.md 1절, 실배선 확인 완료)
#define PIN_CAN_TX 1
#define PIN_CAN_RX 2

// ─────────────────────────────────────────────────────────────────────
// CAN (CAN_프로토콜.md 3~4절)
// ─────────────────────────────────────────────────────────────────────
#define ID_ESTOP      0x010   // any  -> all
#define ID_CMD        0x080   // main -> all   제어 명령
#define ID_TELEM_BASE 0x090   // sub n -> main (0x091~0x093)

#define MODE_VELOCITY 1       // flags bit4..5
#define TEST_AXIS     1       // 1축만 사용

// ─────────────────────────────────────────────────────────────────────
// 시험 조건
// ─────────────────────────────────────────────────────────────────────
// 펄스 목표 휠 속도. 시리얼에서 + / - 로 실시간 조정할 수 있습니다.
//
// 실측 근거 (제작 로그):
//   최대 각가속도  약 138 rad/s²
//   최고 회전      2850 RPM 약 298 rad/s
//
// 반작용 토크는 휠이 "가속하는 동안"에만 나옵니다. 목표에 도달해 등속이
// 되면 토크가 0이 되고 바닥 마찰만 남아 몸체가 멈춥니다. 따라서 목표를
// 높게 잡아 가속 구간을 길게 끄는 것이 반작용을 크게 만드는 방법입니다.
float       pulse_amp        = 150.0f;    // 초기 목표 (약 1430 RPM)
const float PULSE_AMP_MIN    = 10.0f;
const float PULSE_AMP_MAX    = 250.0f;    // 실측 최고 298의 84%. 여유를 둡니다.
const float PULSE_AMP_STEP   = 25.0f;

// 지령 유지 시간. 목표까지 가속하는 데 걸리는 시간보다 길어야 합니다.
//   150 rad/s / 138 rad/s² = 약 1.09초
const unsigned long PULSE_MS      = 1300;  // 지령 유지 시간
const unsigned long REST_MS       = 1800;  // 지령 0 유지 (휠 감속 + 몸체 정지)
const int           PULSE_PAIRS   = 4;     // (+,-) 쌍의 반복 횟수 -> 총 8펄스

// 상관 계산에 쓸 구간의 상한. 지령이 바뀐 뒤 이 시간이 지나면 더 모으지
// 않습니다. 모터가 목표에 도달하지 못하고 포화된 경우, 등속이 아닌데도
// 계속 모으는 것을 막기 위한 안전장치입니다.
const unsigned long MAX_TRANSIENT_MS = 1500;

// 휠 속도가 지령에 이만큼 근접하면 가속이 끝난 것으로 봅니다.
// 각운동량 보존은 휠 속도가 변하는 동안에만 성립합니다. 등속 구간에서는
// 바닥 마찰이 몸체 운동량을 빼앗아 가므로 모델이 깨집니다.
const float RAMP_DONE_TOL_RAD_S = 5.0f;

// ===== 안정화 판정 =====
const float         STABLE_THRESHOLD_DPS = 1.0f;   // 이 아래면 정지로 봄
const unsigned long STABLE_HOLD_MS       = 2000;   // 연속 유지되어야 하는 시간
const unsigned long STABILIZE_TIMEOUT_MS = 60000;

// ===== 바이어스 측정 =====
const int CALIB_SAMPLES = 400;

// ===== 서브 보드 대기 =====
// 서브는 initFOC 정렬 + 정지 확인 때문에 부팅이 수십 초 걸릴 수 있습니다.
const unsigned long SUB_WAIT_TIMEOUT_MS = 60000;

// ===== 주기 =====
const unsigned long TICK_US            = 5000;   // 200Hz. 0x080 송신 주기와 동일
const unsigned long PROGRESS_PERIOD_MS = 1000;

// ===== 판정 기준 =====
const float R_AXIS_STRONG  = 0.90f;   // 모터 축으로 인정할 최소 |r|
const float R_AXIS_CROSS   = 0.30f;   // 다른 축이 이보다 크면 축 정렬 경고
const float REPEAT_TOL_PCT = 20.0f;   // 펄스별 응답 편차 허용치 (%)

// ─────────────────────────────────────────────────────────────────────
// IMU 레지스터 (06 테스트와 동일)
// ─────────────────────────────────────────────────────────────────────
#define REG_WHO_AM_I             0x75
#define WHOAMI_ICM42688          0x47
#define WHOAMI_MPU6500           0x70

#define ICM_DEVICE_CONFIG        0x11
#define ICM_TEMP_DATA1           0x1D
#define ICM_PWR_MGMT0            0x4E
#define ICM_GYRO_CONFIG0         0x4F
#define ICM_ACCEL_CONFIG0        0x50
#define ICM_GYRO_ACCEL_CONFIG0   0x52

#define MPU_SMPLRT_DIV        0x19
#define MPU_CONFIG            0x1A
#define MPU_GYRO_CONFIG       0x1B
#define MPU_ACCEL_CONFIG      0x1C
#define MPU_ACCEL_CONFIG2     0x1D
#define MPU_ACCEL_XOUT_H      0x3B
#define MPU_USER_CTRL         0x6A
#define MPU_PWR_MGMT_1        0x6B
#define MPU_PWR_MGMT_2        0x6C

const float ACCEL_SCALE_G  = 1.0f / 2048.0f;    // ±16g
const float GYRO_SCALE_DPS = 1.0f / 16.4f;      // ±2000 dps

// ─────────────────────────────────────────────────────────────────────
// IMU_2 축 부호
// ─────────────────────────────────────────────────────────────────────
// 06 테스트 보고서의 "6. 상위 코드에 반영할 값"에서 그대로 복사해 넣으세요.
// 두 센서를 같은 방향으로 맞춘 뒤 평균을 내면 노이즈가 줄어듭니다.
const float AXIS_SIGN_2[3] = { -1.0f, +1.0f, -1.0f };

// ─────────────────────────────────────────────────────────────────────
// 센서
// ─────────────────────────────────────────────────────────────────────
enum ImuType { IMU_NONE, IMU_ICM42688, IMU_MPU6500 };

// 진행 단계.
// [주의] 이 enum은 반드시 첫 함수 정의보다 위에 있어야 합니다. Arduino IDE가
// 자동 생성하는 함수 프로토타입을 첫 함수 앞에 끼워 넣기 때문에, enterPhase(Phase)
// 같은 함수의 프로토타입이 enum 정의보다 먼저 나오면 컴파일이 실패합니다.
enum Phase {
  PH_WAIT_SUB,      // 서브 보드 첫 응답 대기
  PH_STABILIZE,     // 바닥 안정화 대기
  PH_BIAS,          // 자이로 바이어스 측정
  PH_PULSE,         // 펄스 시퀀스
  PH_DONE,          // 보고서 출력 후 정지
  PH_ABORT          // 사용자 중단 / 실패
};

struct Imu {
  uint8_t cs_pin;
  const char* label;
  ImuType type;
  uint8_t whoami;
  float bias[3];
};

Imu imu[2] = {
  { PIN_CS_1, "IMU_1", IMU_NONE, 0, { 0, 0, 0 } },
  { PIN_CS_2, "IMU_2", IMU_NONE, 0, { 0, 0, 0 } },
};

int  imu_ok    = 0;
bool bias_valid = false;

// ─────────────────────────────────────────────────────────────────────
// 통계 누적기 (06 테스트와 동일한 구조)
// ─────────────────────────────────────────────────────────────────────
struct PairStat {
  uint32_t n;
  double s1, s2, s11, s22, s12;
};

// x = 휠 속도(rad/s), y = 몸체 각속도(dps). 축별로 따로 모읍니다.
PairStat resp_stat[3];

void statAdd(PairStat& s, double x, double y) {
  s.n++;
  s.s1  += x;      s.s2  += y;
  s.s11 += x * x;  s.s22 += y * y;  s.s12 += x * y;
}
void statClear(PairStat& s) { s.n = 0; s.s1 = s.s2 = s.s11 = s.s22 = s.s12 = 0; }

static inline double varX(const PairStat& s) {
  if (s.n < 2) return 0;
  return (s.s11 - s.s1 * s.s1 / s.n) / (s.n - 1);
}
static inline double varY(const PairStat& s) {
  if (s.n < 2) return 0;
  return (s.s22 - s.s2 * s.s2 / s.n) / (s.n - 1);
}
static inline double covXY(const PairStat& s) {
  if (s.n < 2) return 0;
  return (s.s12 - s.s1 * s.s2 / s.n) / (s.n - 1);
}
double corrCoef(const PairStat& s) {
  double vx = varX(s), vy = varY(s);
  if (vx <= 0 || vy <= 0) return 0;
  return covXY(s) / sqrt(vx * vy);
}
double lsSlope(const PairStat& s) {
  double vx = varX(s);
  if (vx <= 0) return 0;
  return covXY(s) / vx;
}
double residRms(const PairStat& s) {
  double vx = varX(s), vy = varY(s), cxy = covXY(s);
  if (vx <= 0) return 0;
  double ssres = vy - cxy * cxy / vx;
  if (ssres < 0) ssres = 0;
  return sqrt(ssres);
}

// ─────────────────────────────────────────────────────────────────────
// 펄스 기록
// ─────────────────────────────────────────────────────────────────────
const int MAX_PULSES = PULSE_PAIRS * 2;

struct PulseRec {
  float    target;          // 지령 (rad/s)
  float    wheel_peak;      // 휠 실측 최대 속도 (rad/s)
  float    body_peak;       // 몸체 각속도 최대 크기 (dps, 부호 유지)
  int32_t  delay_ms;        // 지령 변화 -> 몸체 반응 시작 (-1 = 반응 없음)
  bool     responded;
};

PulseRec pulse[MAX_PULSES];
int pulse_count = 0;

// ─────────────────────────────────────────────────────────────────────
// 진행 상태 (enum Phase 정의는 위쪽 센서 절에 있습니다)
// ─────────────────────────────────────────────────────────────────────
Phase phase = PH_WAIT_SUB;

unsigned long phase_start_ms = 0;
unsigned long test_start_ms  = 0;
unsigned long last_tick_us   = 0;
unsigned long last_progress_ms = 0;

// 안정화 판정용
unsigned long stable_since_ms = 0;
float stable_noise_dps = 0;

// 펄스 진행
int           pulse_idx      = 0;      // 현재 펄스 번호
bool          in_pulse       = false;  // true = 지령 인가 중, false = 휴지
unsigned long segment_start_ms = 0;
unsigned long last_edge_ms   = 0;      // 마지막 지령 변화 시각
float         cmd_target     = 0;      // 현재 송신 중인 지령

// 모터 상태 (텔레메트리)
uint8_t  seq_counter    = 0;
bool     sub_seen       = false;
float    wheel_vel      = 0;           // rad/s
uint32_t telem_count    = 0;
uint32_t telem_lost     = 0;
uint8_t  telem_status   = 0;
unsigned long last_telem_ms = 0;

// 마지막 표본 (출력용)
float body_w[3] = { 0, 0, 0 };         // 두 센서 평균, 바이어스 보정 후 (dps)
float body_w1[3], body_w2[3];

// ─────────────────────────────────────────────────────────────────────
// IMU SPI
// ─────────────────────────────────────────────────────────────────────
uint8_t readReg(uint8_t cs, uint8_t reg) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  return v;
}

void readRegs(uint8_t cs, uint8_t reg, uint8_t* buf, uint8_t len) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
  SPI.transfer(reg | 0x80);
  for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

void writeReg(uint8_t cs, uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

ImuType detectSensor(Imu& s) {
  uint8_t id = 0;
  for (int i = 0; i < 4; i++) {
    id = readReg(s.cs_pin, REG_WHO_AM_I);
    if (id == WHOAMI_ICM42688 || id == WHOAMI_MPU6500) break;
    delay(10);
  }
  s.whoami = id;
  if (id == WHOAMI_ICM42688) return IMU_ICM42688;
  if (id == WHOAMI_MPU6500)  return IMU_MPU6500;
  return IMU_NONE;
}

void initICM42688(uint8_t cs) {
  writeReg(cs, ICM_DEVICE_CONFIG, 0x01);
  delay(10);
  writeReg(cs, ICM_GYRO_CONFIG0,  0x06);        // ±2000 dps, 1kHz
  writeReg(cs, ICM_ACCEL_CONFIG0, 0x06);        // ±16g, 1kHz
  // UI 필터 25Hz. 200Hz로 읽으므로 나이퀴스트가 100Hz이고, 모터 진동이
  // 그보다 높은 대역에 실려 있어 좁게 잡습니다.
  writeReg(cs, ICM_GYRO_ACCEL_CONFIG0, 0x77);
  writeReg(cs, ICM_PWR_MGMT0, 0x0F);
  delay(1);
  delay(50);
}

void initMPU6500(uint8_t cs) {
  writeReg(cs, MPU_PWR_MGMT_1, 0x80);
  delay(100);
  writeReg(cs, MPU_PWR_MGMT_1, 0x01);
  delay(10);
  writeReg(cs, MPU_USER_CTRL,     0x10);        // I2C 차단 (SPI 전용)
  writeReg(cs, MPU_PWR_MGMT_2,    0x00);
  writeReg(cs, MPU_CONFIG,        0x04);        // DLPF 20Hz
  writeReg(cs, MPU_SMPLRT_DIV,    0x00);
  writeReg(cs, MPU_GYRO_CONFIG,   0x18);        // ±2000 dps
  writeReg(cs, MPU_ACCEL_CONFIG,  0x18);        // ±16g
  writeReg(cs, MPU_ACCEL_CONFIG2, 0x04);        // DLPF 20Hz
  delay(50);
}

static inline int16_t be16(const uint8_t* p) {
  return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

void readGyro(const Imu& s, float g[3]) {
  uint8_t buf[14];
  if (s.type == IMU_ICM42688) {
    readRegs(s.cs_pin, ICM_TEMP_DATA1, buf, 14);
    for (int i = 0; i < 3; i++) g[i] = (float)be16(&buf[8 + i * 2]) * GYRO_SCALE_DPS;
  } else {
    readRegs(s.cs_pin, MPU_ACCEL_XOUT_H, buf, 14);
    for (int i = 0; i < 3; i++) g[i] = (float)be16(&buf[8 + i * 2]) * GYRO_SCALE_DPS;
  }
}

// 두 센서를 같은 방향으로 맞춘 뒤 평균을 냅니다.
// 두 센서의 노이즈가 서로 독립이면 평균으로 노이즈가 1/sqrt(2)로 줄어듭니다.
void readBody() {
  readGyro(imu[0], body_w1);
  if (imu_ok == 2) readGyro(imu[1], body_w2);

  for (int i = 0; i < 3; i++) {
    float w1 = body_w1[i] - imu[0].bias[i];
    if (imu_ok == 2) {
      float w2 = (body_w2[i] - imu[1].bias[i]) * AXIS_SIGN_2[i];
      body_w[i] = 0.5f * (w1 + w2);
    } else {
      body_w[i] = w1;
    }
  }
}

float bodyMag() {
  return fabsf(body_w[0]) + fabsf(body_w[1]) + fabsf(body_w[2]);
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

// 1축만 enable 합니다. 나머지 축 목표는 0으로 보냅니다.
void sendCommand(float target, bool enable) {
  twai_message_t m;
  m.flags = 0;
  m.identifier = ID_CMD;
  m.data_length_code = 8;

  uint8_t flags = 0;
  if (enable) flags |= (1 << (TEST_AXIS - 1));      // 1축만
  flags |= (MODE_VELOCITY & 0x03) << 4;

  int16_t t = (int16_t)(target * 10.0f);            // rad/s x10

  m.data[0] = seq_counter++;
  m.data[1] = flags;
  m.data[2] = t & 0xFF;  m.data[3] = (t >> 8) & 0xFF;   // 1축
  m.data[4] = 0;         m.data[5] = 0;                 // 2축
  m.data[6] = 0;         m.data[7] = 0;                 // 3축

  twai_transmit(&m, 0);
}

void sendEstop(uint8_t reason) {
  twai_message_t m;
  m.flags = 0;
  m.identifier = ID_ESTOP;
  m.data_length_code = 8;
  m.data[0] = reason;
  m.data[1] = 0;                                    // source = 메인
  for (int i = 2; i < 8; i++) m.data[i] = 0;
  twai_transmit(&m, 0);
}

void pollTelemetry() {
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) {
    if ((m.identifier & 0xFF0) != ID_TELEM_BASE) continue;
    if (m.data_length_code != 8) continue;
    if ((int)(m.identifier & 0x0F) != TEST_AXIS) continue;

    sub_seen = true;
    telem_count++;
    telem_status |= m.data[1];
    last_telem_ms = millis();

    int16_t v = (int16_t)((uint16_t)m.data[4] | ((uint16_t)m.data[5] << 8));
    wheel_vel = v / 10.0f;                          // rad/s
  }
}

// ─────────────────────────────────────────────────────────────────────
// 출력
// ─────────────────────────────────────────────────────────────────────
const char* typeName(ImuType t) {
  if (t == IMU_ICM42688) return "ICM-42688-P";
  if (t == IMU_MPU6500)  return "MPU-6500";
  return "UNKNOWN";
}

void printHelp() {
  char buf[110];
  Serial.println(F("---- 명령 ----"));
  Serial.println(F("  x : 즉시 정지 (E-STOP 송신)"));
  Serial.println(F("  + : 목표 휠 속도 올리기 (테스트 재시작됨)"));
  Serial.println(F("  - : 목표 휠 속도 내리기 (테스트 재시작됨)"));
  Serial.println(F("  r : 테스트 재시작"));
  Serial.println(F("  h : 도움말"));
  snprintf(buf, sizeof(buf), "  현재 목표: ±%.0f rad/s (약 %.0f RPM), 조정 단위 %.0f, 상한 %.0f",
           pulse_amp, pulse_amp * 60.0f / (2.0f * (float)PI), PULSE_AMP_STEP, PULSE_AMP_MAX);
  Serial.println(buf);
}

void printRule() {
  Serial.println(F("--------------------------------------------------------------"));
}

// ─────────────────────────────────────────────────────────────────────
// 보고서
// ─────────────────────────────────────────────────────────────────────
void printReport() {
  char buf[190];
  const char* axis_name[3] = { "X", "Y", "Z" };

  int n_fail = 0, n_warn = 0;

  Serial.println();
  Serial.println(F("=============================================================="));
  Serial.println(F("             모터 - 센서 상관관계 테스트 보고서"));
  Serial.println(F("=============================================================="));

  // ---- 시험 조건 ----
  snprintf(buf, sizeof(buf), " 대상   : %d축 (X축으로 정의)", TEST_AXIS);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), " 센서 1 : %s (CS=GPIO%d)", typeName(imu[0].type), imu[0].cs_pin);
  Serial.println(buf);
  if (imu_ok == 2) {
    snprintf(buf, sizeof(buf), " 센서 2 : %s (CS=GPIO%d), 두 센서 평균 사용",
             typeName(imu[1].type), imu[1].cs_pin);
  } else {
    snprintf(buf, sizeof(buf), " 센서 2 : 미검출. 센서 1 단독으로 측정했습니다.");
    n_warn++;
  }
  Serial.println(buf);
  snprintf(buf, sizeof(buf), " 지령   : ±%.0f rad/s (약 %.0f RPM) 속도 모드, 펄스 %lums / 휴지 %lums, %d회",
           pulse_amp, pulse_amp * 60.0f / (2.0f * (float)PI), PULSE_MS, REST_MS, pulse_count);
  Serial.println(buf);
  Serial.println(F(" 상관구간: 휠이 가속/감속하는 동안만 사용 (등속 구간 제외)"));
  snprintf(buf, sizeof(buf), " 텔레메트리: %lu 프레임 수신", (unsigned long)telem_count);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), " 소요   : %.1f 초", (millis() - test_start_ms) / 1000.0f);
  Serial.println(buf);

  // ---- 1. 안정화 ----
  printRule();
  Serial.println(F(" 1. 바닥 안정화"));
  printRule();
  snprintf(buf, sizeof(buf), "   안정화 후 잔류 자이로 노이즈: %.3f dps", stable_noise_dps);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "   판정 문턱 %.1f dps 아래로 %lums 연속 유지 확인",
           STABLE_THRESHOLD_DPS, STABLE_HOLD_MS);
  Serial.println(buf);
  Serial.println(F("   자이로 바이어스 (dps):"));
  for (int s = 0; s < imu_ok; s++) {
    snprintf(buf, sizeof(buf), "     %s  X %+.4f  Y %+.4f  Z %+.4f",
             imu[s].label, imu[s].bias[0], imu[s].bias[1], imu[s].bias[2]);
    Serial.println(buf);
  }

  // ---- 2. 축별 상관 ----
  printRule();
  Serial.println(F(" 2. 축별 상관 (과도 구간, x = 휠 속도 rad/s, y = 몸체 각속도 dps)"));
  printRule();
  Serial.println(F("   축   표본        k          r         resid"));

  int   motor_axis = -1;
  double best_r = 0;
  for (int i = 0; i < 3; i++) {
    const PairStat& s = resp_stat[i];
    double k = lsSlope(s), r = corrCoef(s), e = residRms(s);
    snprintf(buf, sizeof(buf), "   %s  %5lu   %+9.4f  %+9.4f  %8.3f",
             axis_name[i], (unsigned long)s.n, k, r, e);
    Serial.println(buf);
    if (fabs(r) > fabs(best_r)) { best_r = r; motor_axis = i; }
  }
  Serial.println(F("   k = 몸체 각속도 / 휠 속도. 음수여야 반작용이 맞습니다."));
  Serial.println(F("   r = 반작용 모델이 얼마나 잘 맞는지. 모터 축에서만 1에 가까워야 합니다."));

  // ---- 3. 모터 축 판정 ----
  printRule();
  Serial.println(F(" 3. 모터 축 판정"));
  printRule();
  if (motor_axis < 0 || fabs(best_r) < R_AXIS_STRONG) {
    Serial.println(F("   [FAIL] 어느 축에서도 뚜렷한 반작용이 관측되지 않았습니다."));
    Serial.println(F("          - 모터가 실제로 돌았는지 (아래 4절 휠 속도 확인)"));
    Serial.println(F("          - 반작용 휠이 장착되어 있는지"));
    Serial.println(F("          - 조립체가 바닥에 눌려 회전하지 못하는지 (마찰 과다)"));
    Serial.println(F("          - 지령을 더 크게 (+ 키로 목표 휠 속도 상향)"));
    n_fail++;
  } else {
    double k = lsSlope(resp_stat[motor_axis]);
    snprintf(buf, sizeof(buf), "   모터 회전축 = IMU %s축   (|r| = %.4f)",
             axis_name[motor_axis], fabs(best_r));
    Serial.println(buf);

    if (k < 0) {
      Serial.println(F("   반작용 부호: 휠 + 방향 회전 -> 몸체 - 방향 회전. 정상입니다."));
    } else {
      Serial.println(F("   [WARN] 휠과 몸체가 같은 방향으로 돕니다. 반작용이라면 반대여야 합니다."));
      Serial.println(F("          휠이 몸체에 마찰로 끌려가고 있거나(베어링 고착),"));
      Serial.println(F("          텔레메트리 속도 부호가 반대로 정의되어 있을 수 있습니다."));
      n_warn++;
    }

    snprintf(buf, sizeof(buf), "   관성비 I_wheel / I_body = %.5f", fabs(k) * (float)(PI / 180.0));
    Serial.println(buf);
    Serial.println(F("   (k는 dps/(rad/s) 단위이므로 rad 단위로 환산한 값입니다.)"));

    // 다른 축으로 새는 성분 확인
    bool cross = false;
    for (int i = 0; i < 3; i++) {
      if (i == motor_axis) continue;
      if (fabs(corrCoef(resp_stat[i])) > R_AXIS_CROSS) cross = true;
    }
    if (cross) {
      Serial.println(F("   [WARN] 다른 축에도 상관이 큽니다. 모터 회전축이 IMU 축과 어긋나 있거나"));
      Serial.println(F("          조립체가 바닥에서 미끄러지며 복합 운동을 하고 있습니다."));
      n_warn++;
    }
  }

  // ---- 4. 펄스별 응답 ----
  printRule();
  Serial.println(F(" 4. 펄스별 응답"));
  printRule();
  Serial.println(F("   #   지령      휠피크    몸체피크   지연"));

  double sum_abs = 0, max_abs = 0, min_abs = 1e9;
  int    n_resp = 0;
  double sum_delay = 0;
  int    n_delay = 0;

  for (int i = 0; i < pulse_count; i++) {
    const PulseRec& p = pulse[i];
    if (p.responded) {
      snprintf(buf, sizeof(buf), "  %2d  %+7.1f  %+8.1f  %+9.2f  %5ld ms",
               i + 1, p.target, p.wheel_peak, p.body_peak, (long)p.delay_ms);
      double a = fabs(p.body_peak);
      sum_abs += a; n_resp++;
      if (a > max_abs) max_abs = a;
      if (a < min_abs) min_abs = a;
      if (p.delay_ms >= 0) { sum_delay += p.delay_ms; n_delay++; }
    } else {
      snprintf(buf, sizeof(buf), "  %2d  %+7.1f  %+8.1f  %+9.2f      -    반응없음",
               i + 1, p.target, p.wheel_peak, p.body_peak);
    }
    Serial.println(buf);
  }
  Serial.println(F("   단위: 지령/휠 rad/s, 몸체 dps. 지연 = 지령 변화 -> 몸체 반응 시작."));

  if (n_resp >= 2) {
    double mean_abs = sum_abs / n_resp;
    double spread = (mean_abs > 0) ? (100.0 * (max_abs - min_abs) / mean_abs) : 0;
    snprintf(buf, sizeof(buf), "   몸체 응답 크기: 평균 %.2f dps, 편차 %.1f%% (최소 %.2f / 최대 %.2f)",
             mean_abs, spread, min_abs, max_abs);
    Serial.println(buf);
    if (spread > REPEAT_TOL_PCT) {
      Serial.println(F("   [WARN] 펄스마다 응답 크기가 많이 다릅니다. 바닥 마찰이 일정하지 않거나"));
      Serial.println(F("          조립체가 매번 다른 자세에서 출발하고 있습니다."));
      n_warn++;
    }
  }
  if (n_delay > 0) {
    snprintf(buf, sizeof(buf), "   평균 반응 지연: %.0f ms", sum_delay / n_delay);
    Serial.println(buf);
    Serial.println(F("   여기에는 CAN 왕복(약 0.5ms), 모터 전류 상승, 휠 관성 가속,"));
    Serial.println(F("   그리고 바닥 정지마찰을 이기는 시간이 모두 포함되어 있습니다."));
  }

  // ---- 5. 종합 판정 ----
  printRule();
  Serial.println(F(" 5. 종합 판정"));
  printRule();
  if (n_fail > 0) {
    Serial.println(F("   [FAIL] 모터와 센서의 상관을 확인하지 못했습니다. 위 항목을 확인하세요."));
  } else if (n_warn > 0) {
    snprintf(buf, sizeof(buf), "   [WARN] 반작용은 확인되었으나 경고 %d건이 있습니다.", n_warn);
    Serial.println(buf);
  } else {
    Serial.println(F("   [PASS] 모터 지령과 몸체 반작용의 상관이 정상적으로 확인되었습니다."));
  }

  // ---- 6. 상위 코드에 반영할 값 ----
  printRule();
  Serial.println(F(" 6. 상위 코드에 반영할 값 (복사해서 사용)"));
  printRule();
  if (motor_axis >= 0) {
    snprintf(buf, sizeof(buf), "   // 1축(X축) 몸체 각속도를 읽을 IMU 축 인덱스 (0=X, 1=Y, 2=Z)");
    Serial.println(buf);
    snprintf(buf, sizeof(buf), "   const int   AXIS1_GYRO_INDEX = %d;   // IMU %s축",
             motor_axis, axis_name[motor_axis]);
    Serial.println(buf);
    double k = lsSlope(resp_stat[motor_axis]);
    Serial.println(F("   // 모터 + 지령에 대해 몸체가 도는 방향. 제어 부호를 여기에 맞춥니다."));
    snprintf(buf, sizeof(buf), "   const float AXIS1_REACTION_SIGN = %+.0ff;", (k < 0) ? -1.0f : 1.0f);
    Serial.println(buf);
    Serial.println(F("   // 휠 속도 1 rad/s 변화당 몸체 각속도 변화 (rad/s per rad/s)"));
    snprintf(buf, sizeof(buf), "   const float AXIS1_INERTIA_RATIO = %.5ff;",
             fabs(k) * (float)(PI / 180.0));
    Serial.println(buf);
  } else {
    Serial.println(F("   측정 실패로 값을 산출하지 못했습니다."));
  }
  Serial.println(F("   // 자이로 바이어스 (dps)"));
  for (int s = 0; s < imu_ok; s++) {
    snprintf(buf, sizeof(buf), "   const float GYRO_BIAS_%d[3] = { %+.4ff, %+.4ff, %+.4ff };",
             s + 1, imu[s].bias[0], imu[s].bias[1], imu[s].bias[2]);
    Serial.println(buf);
  }

  Serial.println(F("=============================================================="));
  Serial.println(F(" 테스트 종료. 모터 정지됨.  r = 다시 측정,  h = 도움말"));
  Serial.println(F("=============================================================="));
  Serial.println();
}

// ─────────────────────────────────────────────────────────────────────
// 단계 전환
// ─────────────────────────────────────────────────────────────────────
void enterPhase(Phase p) {
  phase = p;
  phase_start_ms = millis();
}

void restartTest() {
  for (int i = 0; i < 3; i++) statClear(resp_stat[i]);
  for (int i = 0; i < MAX_PULSES; i++) {
    pulse[i].target = pulse[i].wheel_peak = pulse[i].body_peak = 0;
    pulse[i].delay_ms = -1;
    pulse[i].responded = false;
  }
  pulse_count = 0;
  pulse_idx   = 0;
  in_pulse    = false;
  cmd_target  = 0;
  bias_valid  = false;
  for (int s = 0; s < 2; s++)
    for (int i = 0; i < 3; i++) imu[s].bias[i] = 0;

  telem_count = 0;
  telem_lost  = 0;
  telem_status = 0;
  stable_since_ms = 0;
  stable_noise_dps = 0;

  test_start_ms = millis();
  enterPhase(sub_seen ? PH_STABILIZE : PH_WAIT_SUB);

  Serial.println();
  Serial.println(F("=== 테스트 시작 ==="));
  if (phase == PH_STABILIZE)
    Serial.println(F("조립체를 바닥에 두고 손을 떼세요. 안정될 때까지 기다립니다."));
}

void abortTest(const char* why) {
  sendEstop(0x00);
  cmd_target = 0;
  enterPhase(PH_ABORT);
  Serial.println();
  Serial.print(F("[ABORT] "));
  Serial.println(why);
  Serial.println(F("E-STOP을 송신했습니다. 서브는 전원 재인가 전까지 출력을 차단합니다."));
  Serial.println(F("r 로 다시 시작할 수 있으나, E-STOP 해제를 위해 서브 전원을 껐다 켜야 합니다."));
}

// ─────────────────────────────────────────────────────────────────────
// 각 단계 처리 (200Hz 틱 안에서 호출)
// ─────────────────────────────────────────────────────────────────────
void tickWaitSub() {
  cmd_target = 0;
  if (sub_seen) {
    Serial.println(F("서브 보드 응답 확인."));
    Serial.println();
    Serial.println(F("--- 바닥 안정화 대기 ---"));
    Serial.println(F("조립체를 바닥에 두고 손을 떼세요."));
    enterPhase(PH_STABILIZE);
    return;
  }
  if (millis() - phase_start_ms > SUB_WAIT_TIMEOUT_MS) {
    abortTest("서브 보드 텔레메트리를 받지 못했습니다. CAN 배선과 서브 전원을 확인하세요.");
  }
}

void tickStabilize() {
  cmd_target = 0;
  float mag = bodyMag();
  unsigned long now = millis();

  if (mag > STABLE_THRESHOLD_DPS) {
    stable_since_ms = 0;              // 흔들리면 처음부터 다시
    return;
  }
  if (stable_since_ms == 0) stable_since_ms = now;

  if (now - stable_since_ms >= STABLE_HOLD_MS) {
    Serial.println(F("안정화 확인. 자이로 바이어스를 측정합니다. 계속 건드리지 마세요."));
    enterPhase(PH_BIAS);
    return;
  }
  if (now - phase_start_ms > STABILIZE_TIMEOUT_MS) {
    abortTest("안정화되지 않았습니다. 바닥이 흔들리거나 진동원이 있습니다.");
  }
}

// 바이어스 측정은 틱 안에서 나눠 하지 않고 한 번에 끝냅니다.
// 이 구간은 모터가 정지 상태라 200Hz 송신이 잠시 끊겨도 안전합니다.
// (서브는 15ms 워치독으로 출력을 차단할 뿐이고 이미 0 지령입니다.)
void doBiasMeasure() {
  double sum[2][3] = { { 0, 0, 0 }, { 0, 0, 0 } };
  double sq[3] = { 0, 0, 0 };
  float g[3];

  for (int n = 0; n < CALIB_SAMPLES; n++) {
    for (int s = 0; s < imu_ok; s++) {
      readGyro(imu[s], g);
      for (int i = 0; i < 3; i++) sum[s][i] += g[i];
    }
    // 잔류 노이즈는 센서 1 기준으로 봅니다.
    readGyro(imu[0], g);
    for (int i = 0; i < 3; i++) sq[i] += (double)g[i] * g[i];

    sendCommand(0, false);            // 워치독 유지
    delay(3);
    if (n % 100 == 0) Serial.print('.');
  }
  Serial.println();

  for (int s = 0; s < imu_ok; s++)
    for (int i = 0; i < 3; i++)
      imu[s].bias[i] = (float)(sum[s][i] / CALIB_SAMPLES);
  bias_valid = true;

  // 바이어스를 뺀 뒤의 표준편차를 잔류 노이즈로 씁니다.
  double worst = 0;
  for (int i = 0; i < 3; i++) {
    double m = imu[0].bias[i];
    double v = sq[i] / CALIB_SAMPLES - m * m;
    if (v < 0) v = 0;
    if (sqrt(v) > worst) worst = sqrt(v);
  }
  stable_noise_dps = (float)worst;

  char buf[130];
  for (int s = 0; s < imu_ok; s++) {
    snprintf(buf, sizeof(buf), "  %s 바이어스: X %+.4f  Y %+.4f  Z %+.4f dps",
             imu[s].label, imu[s].bias[0], imu[s].bias[1], imu[s].bias[2]);
    Serial.println(buf);
  }
  snprintf(buf, sizeof(buf), "  잔류 노이즈: %.4f dps", stable_noise_dps);
  Serial.println(buf);

  Serial.println();
  Serial.println(F("--- 펄스 시퀀스 시작 ---"));
  Serial.println(F("[안전] 조립체가 바닥에서 실제로 회전합니다. 주변을 비우세요."));
  Serial.println();

  pulse_idx = 0;
  in_pulse  = false;
  segment_start_ms = millis();
  last_edge_ms = millis();
  enterPhase(PH_PULSE);
}

void tickPulse() {
  unsigned long now = millis();

  // ---- 구간 전환 ----
  unsigned long seg_len = in_pulse ? PULSE_MS : REST_MS;
  if (now - segment_start_ms >= seg_len) {
    if (in_pulse) {
      // 펄스 끝 -> 휴지. 방금 끝난 펄스를 확정합니다.
      in_pulse = false;
      cmd_target = 0;
      pulse_idx++;
      pulse_count = pulse_idx;
    } else {
      // 휴지 끝 -> 다음 펄스. 다 돌았으면 여기서 끝냅니다.
      if (pulse_idx >= MAX_PULSES) {
        cmd_target = 0;
        enterPhase(PH_DONE);
        printReport();
        return;
      }
      in_pulse = true;
      // 짝수 번째는 +, 홀수 번째는 - 로 번갈아 줍니다.
      // 한 방향으로만 주면 몸체가 한쪽으로 계속 밀려 원위치를 잃습니다.
      cmd_target = (pulse_idx % 2 == 0) ? pulse_amp : -pulse_amp;

      pulse[pulse_idx].target     = cmd_target;
      pulse[pulse_idx].wheel_peak = 0;
      pulse[pulse_idx].body_peak  = 0;
      pulse[pulse_idx].delay_ms   = -1;
      pulse[pulse_idx].responded  = false;

      char buf[80];
      snprintf(buf, sizeof(buf), "  펄스 %d/%d  지령 %+.1f rad/s",
               pulse_idx + 1, MAX_PULSES, cmd_target);
      Serial.println(buf);
    }
    segment_start_ms = now;
    last_edge_ms = now;
  }

  // ---- 휠이 가속/감속하는 동안에만 상관 표본을 모읍니다 ----
  // 각운동량 보존은 휠 속도가 변하는 동안에만 성립합니다. 등속 구간에서는
  // 반작용 토크가 0이라 바닥 마찰이 몸체를 세우고, 그 구간을 함께 넣으면
  // 기울기가 0쪽으로 끌려갑니다.
  //
  // 고정 시간창 대신 "지령에 아직 도달하지 못했는가"로 판정합니다. 지령을
  // 높이면 가속 구간이 길어지는데, 고정 시간창은 그 뒷부분을 버리게 됩니다.
  bool ramping = fabsf(wheel_vel - cmd_target) > RAMP_DONE_TOL_RAD_S;
  if (ramping && (now - last_edge_ms <= MAX_TRANSIENT_MS)) {
    for (int i = 0; i < 3; i++) statAdd(resp_stat[i], wheel_vel, body_w[i]);
  }

  // ---- 펄스 중 최댓값과 반응 지연 기록 ----
  if (in_pulse && pulse_idx < MAX_PULSES) {
    PulseRec& p = pulse[pulse_idx];

    if (fabsf(wheel_vel) > fabsf(p.wheel_peak)) p.wheel_peak = wheel_vel;

    // 몸체 응답은 아직 어느 축이 모터 축인지 모르므로 가장 크게 움직인
    // 축의 값을 기록합니다. 축 판정은 보고서에서 상관으로 따로 합니다.
    float best = 0;
    for (int i = 0; i < 3; i++) if (fabsf(body_w[i]) > fabsf(best)) best = body_w[i];
    if (fabsf(best) > fabsf(p.body_peak)) p.body_peak = best;

    // 반응 시작 = 잔류 노이즈의 5배를 처음 넘은 시점
    if (!p.responded) {
      float trig = stable_noise_dps * 5.0f;
      if (trig < 0.5f) trig = 0.5f;               // 노이즈가 너무 작을 때의 하한
      if (fabsf(best) > trig) {
        p.responded = true;
        p.delay_ms  = (int32_t)(now - segment_start_ms);
      }
    }
  }

}

// ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("=============================================="));
  Serial.println(F(" 모터 - 센서 상관관계 테스트 - MAIN (ESP32-S3)"));
  Serial.print  (F(" 대상 축: ")); Serial.print(TEST_AXIS); Serial.println(F("축 (X축으로 정의)"));
  Serial.print  (F(" IMU SPI: SCK=GPIO")); Serial.print(PIN_SPI_SCLK);
  Serial.print  (F(" MOSI=GPIO"));         Serial.print(PIN_SPI_MOSI);
  Serial.print  (F(" MISO=GPIO"));         Serial.println(PIN_SPI_MISO);
  Serial.print  (F(" CS: IMU_1=GPIO"));    Serial.print(PIN_CS_1);
  Serial.print  (F("  IMU_2=GPIO"));       Serial.println(PIN_CS_2);
  Serial.print  (F(" CAN: TX=GPIO"));      Serial.print(PIN_CAN_TX);
  Serial.print  (F(" RX=GPIO"));           Serial.print(PIN_CAN_RX);
  Serial.println(F(" @500kbit/s"));
  Serial.print  (F(" 지령: ±"));           Serial.print(pulse_amp, 0);
  Serial.print  (F(" rad/s (약 "));        Serial.print(pulse_amp * 60.0f / (2.0f * (float)PI), 0);
  Serial.println(F(" RPM) 속도 모드"));
  Serial.println(F("=============================================="));
  Serial.println(F("[안전] 모터가 실제로 회전하고 조립체가 바닥에서 돕니다."));
  Serial.println(F("       주변을 비우고, x 로 언제든 정지할 수 있습니다."));
  Serial.println(F("=============================================="));

  // ---- IMU ----
  for (int s = 0; s < 2; s++) {
    pinMode(imu[s].cs_pin, OUTPUT);
    digitalWrite(imu[s].cs_pin, HIGH);
  }
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
  delay(10);

  Serial.println(F("IMU 판별 중..."));
  for (int s = 0; s < 2; s++) {
    imu[s].type = detectSensor(imu[s]);
    char buf[90];
    if (imu[s].type == IMU_NONE) {
      snprintf(buf, sizeof(buf), "  %s : 미검출 (WHO_AM_I = 0x%02X)", imu[s].label, imu[s].whoami);
    } else {
      snprintf(buf, sizeof(buf), "  %s : %s (0x%02X)",
               imu[s].label, typeName(imu[s].type), imu[s].whoami);
      imu_ok++;
    }
    Serial.println(buf);
  }
  if (imu_ok == 0) {
    Serial.println(F("[FAIL] IMU가 하나도 없습니다. 06 테스트로 배선을 먼저 확인하세요."));
    while (1) delay(500);
  }
  if (imu_ok == 1)
    Serial.println(F("[WARN] 센서 1개로 진행합니다. 노이즈가 평균 효과 없이 그대로 남습니다."));

  for (int s = 0; s < imu_ok; s++) {
    if (imu[s].type == IMU_ICM42688) initICM42688(imu[s].cs_pin);
    else                             initMPU6500(imu[s].cs_pin);
  }
  Serial.println(F("  IMU 초기화 완료. 자이로 ±2000 dps, 필터 25Hz"));

  // ---- CAN ----
  if (!canInit()) {
    Serial.println(F("[FAIL] CAN 초기화 실패."));
    while (1) delay(500);
  }
  Serial.println(F("  CAN 초기화 완료."));

  Serial.println();
  printHelp();
  Serial.println();
  Serial.println(F("--- 서브 보드 대기 ---"));
  Serial.println(F("서브의 첫 텔레메트리를 기다립니다. initFOC 정렬 때문에 수십 초 걸릴 수 있습니다."));

  test_start_ms  = millis();
  last_tick_us   = micros();
  last_progress_ms = millis();
  enterPhase(PH_WAIT_SUB);
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'x':
        if (phase != PH_ABORT) abortTest("사용자 중단");
        break;
      case 'r':
        restartTest();
        break;
      case '+':
      case '=':                                  // shift 없이 누른 경우도 받습니다
      case '-':
      case '_': {
        // 지령이 바뀌면 이전 표본과 섞을 수 없으므로 테스트를 다시 시작합니다.
        float before = pulse_amp;
        pulse_amp += (c == '+' || c == '=') ? PULSE_AMP_STEP : -PULSE_AMP_STEP;
        if (pulse_amp > PULSE_AMP_MAX) pulse_amp = PULSE_AMP_MAX;
        if (pulse_amp < PULSE_AMP_MIN) pulse_amp = PULSE_AMP_MIN;

        char buf[110];
        snprintf(buf, sizeof(buf), "-> 목표 휠 속도 %.0f -> %.0f rad/s (약 %.0f RPM)",
                 before, pulse_amp, pulse_amp * 60.0f / (2.0f * (float)PI));
        Serial.println(buf);
        if (pulse_amp >= PULSE_AMP_MAX)
          Serial.println(F("   상한입니다. 실측 최고 회전(약 298 rad/s)의 84%로 제한해 두었습니다."));
        restartTest();
        break;
      }
      case 'h':
        printHelp();
        break;
    }
  }
}

void printProgress() {
  char buf[160];
  switch (phase) {
    case PH_WAIT_SUB:
      snprintf(buf, sizeof(buf), "  [대기] 서브 응답 없음. %.0f초 경과",
               (millis() - phase_start_ms) / 1000.0f);
      Serial.println(buf);
      break;
    case PH_STABILIZE: {
      float held = (stable_since_ms > 0) ? (millis() - stable_since_ms) / 1000.0f : 0;
      snprintf(buf, sizeof(buf), "  [안정화] 자이로 %.2f dps (문턱 %.1f)   유지 %.1f/%.1f초",
               bodyMag(), STABLE_THRESHOLD_DPS, held, STABLE_HOLD_MS / 1000.0f);
      Serial.println(buf);
      break;
    }
    case PH_PULSE:
      snprintf(buf, sizeof(buf), "  [펄스] %d/%d   휠 %+.1f rad/s   몸체 X%+.1f Y%+.1f Z%+.1f dps",
               pulse_idx + 1, MAX_PULSES, wheel_vel, body_w[0], body_w[1], body_w[2]);
      Serial.println(buf);
      break;
    default:
      break;
  }
}

void loop() {
  handleSerial();

  // 바이어스 측정은 블로킹으로 한 번에 처리합니다.
  if (phase == PH_BIAS) { doBiasMeasure(); return; }

  unsigned long now_us = micros();
  if (now_us - last_tick_us < TICK_US) return;
  last_tick_us = now_us;

  // ---- 200Hz 틱 ----
  readBody();
  pollTelemetry();

  switch (phase) {
    case PH_WAIT_SUB:  tickWaitSub();  break;
    case PH_STABILIZE: tickStabilize(); break;
    case PH_PULSE:     tickPulse();    break;
    case PH_DONE:      cmd_target = 0; break;
    case PH_ABORT:     cmd_target = 0; break;
    default: break;
  }

  // 워치독 유지를 위해 항상 200Hz로 송신합니다.
  // 중단 상태에서는 enable을 내려 0 지령만 보냅니다.
  bool enable = (phase == PH_PULSE);
  if (phase != PH_ABORT) sendCommand(cmd_target, enable);

  // 텔레메트리가 끊기면 즉시 멈춥니다.
  if (phase == PH_PULSE && millis() - last_telem_ms > 100) {
    abortTest("텔레메트리가 100ms 이상 끊겼습니다. 모터 상태를 알 수 없습니다.");
  }

  if (millis() - last_progress_ms >= PROGRESS_PERIOD_MS) {
    last_progress_ms = millis();
    printProgress();
  }
}
