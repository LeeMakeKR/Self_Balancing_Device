// IMU 2개 동시 읽기 + 정합 분석 테스트 - 메인 컨트롤 보드 (ESP32-S3)
//
// 역할: 같은 축에 배치된 IMU 2개를 동시에 읽어, 두 센서가 서로 어떤 관계로
//       움직이는지를 수치로 뽑아냅니다. 모터는 돌리지 않습니다.
//
// 지원 센서: ICM-42688-P / MPU-6500 (센서마다 개별 자동 판별)
//   두 기종 모두 WHO_AM_I가 레지스터 0x75에 있어 값으로 구분합니다.
//   ICM-42688-P -> 0x47,  MPU-6500 -> 0x70
//
// 라이브러리를 쓰지 않습니다. 두 기종 모두 부팅 시 설정 파일 업로드가 없어
// 레지스터 직접 접근만으로 초기화와 읽기가 끝납니다. 외부 의존성 없음.
//
// 참조 문서: README.md 핀 배정표
//
// ─────────────────────────────────────────────────────────────────────
// 배선 (SPI 3선 공통 + CS 개별)
// ─────────────────────────────────────────────────────────────────────
//   ESP32-S3    센서 핀 표기                        역할
//   ----------  ----------------------------------  ----------------------
//   3V3         VCC                                 전원. 3.3V (5V 금지)
//   GND         GND                                 접지
//   GPIO6       SCL / SCLK / SPC                    SPI 클럭 SCK   (2개 공통)
//   GPIO5       SDA / SDI                           SPI MOSI       (2개 공통)
//   GPIO4       AD0 / SDO                           SPI MISO       (2개 공통)
//   GPIO7       CS                                  칩 선택 (IMU_1)
//   GPIO15      CS                                  칩 선택 (IMU_2)
//   미연결      INT1 / INT2                         이 테스트는 폴링이라 미사용
//
//   [필수] CS는 센서마다 따로여야 합니다. 두 센서의 CS를 묶으면 MISO에서 두 칩이
//          동시에 응답해 데이터가 깨집니다.
//
// ─────────────────────────────────────────────────────────────────────
// 두 센서가 모터 중심축 위에 배치된 것의 의미
// ─────────────────────────────────────────────────────────────────────
//   현재 배치: 모터 중심축을 따라 모터와 자이로 2개가 일직선으로 놓여 있음.
//
//   자이로는 강체의 각속도를 읽습니다. 각속도는 강체 위 어느 지점에서나
//   같으므로, 두 센서가 축 위 다른 위치에 있어도 회전만 시키면 둘 다
//   같은 각속도를 봅니다. 위치 차이는 자이로에 영향을 주지 않습니다.
//
//   장착 방향에 따라 일부 축의 부호가 반대로 읽힙니다(앞뒤로 뒤집어 붙인
//   경우). 어느 축이 뒤집혔는지는 가정하지 않고 측정으로 판정합니다.
//
//   판정은 두 구간을 나눠서 합니다. 정지 상태의 자이로는 노이즈뿐이라
//   부호를 판정할 수 없고, 대신 바이어스와 노이즈 크기를 재기에 좋습니다.
//   반대로 회전하는 구간은 부호와 배율을 판정하기에 좋습니다.
//
// ─────────────────────────────────────────────────────────────────────
// [중요] 운동 구간에서 무엇을 해야 하는가
// ─────────────────────────────────────────────────────────────────────
//   조립체를 "회전"시킬 것. 축을 중심으로 좌우로 비트는 왕복.
////
//   3축을 모두 검증하려면 세 방향의 회전이 각각 필요합니다.
//     1) 모터 중심축을 회전축으로 비틀기  <- 밸런싱 제어에 실제로 쓰이는 축
//     2) 그 축에 수직인 방향으로 기울이기 (앞뒤로 끄덕이듯)
//     3) 나머지 한 방향으로 기울이기      (좌우로 갸웃하듯)
//
//   한 축만 회전시키면 나머지 두 축은 신호가 없어 판정이 불가능합니다.
//   그래서 이 코드는 축마다 따로 표본을 셉니다. 어떤 축이 아직 덜 찼는지
//   진행률에 표시되므로, 그 축의 회전을 추가로 주면 됩니다.
//
//   모터 중심축 하나만 검증해도 충분하다면 f 를 눌러 조기 종료할 수 있습니다.
//   보고서는 표본이 모자란 축을 "판정불가"로 따로 표시합니다.
//
// ─────────────────────────────────────────────────────────────────────
// 출력하는 변수
// ─────────────────────────────────────────────────────────────────────
//   [정지 구간] 축별로
//     bias1, bias2  : 각 센서 자이로 평균 (dps). 적분 드리프트의 원인.
//     sd1, sd2      : 각 센서 자이로 표준편차 (dps). 노이즈 크기.
//     r_still       : 두 센서 노이즈의 상관계수. 0 근처가 정상.
//                     크게 나오면 두 센서가 같은 외란(전원/진동)을 함께 타는 것.
//   [정지 구간] 가속도
//     |a1|, |a2|    : 가속도 벡터 크기 (g). 1.000 근처여야 정상.
//     angle         : 두 가속도 벡터 사이 각도 (deg). 장착 정렬 오차.
//                     같은 방향이면 0, 뒤집어 붙였으면 180 근처.
//
//   [회전 구간] 축별로 (그 축을 회전축으로 삼은 회전이 있었던 표본만)
//     회전크기      : 그 축 각속도의 표준편차 (dps). 얼마나 세게 돌렸는지.
//     SNR           : 회전크기 / resid. 이 값이 작으면 k를 믿을 수 없음.
//     k             : 최소자승 기울기. g2 ≈ k * g1 관계의 계수.
//                     부호가 두 센서의 축 부호 관계, 크기가 배율 차이.
//                     정상이면 +1 또는 -1 근처.
//     r_move        : 피어슨 상관계수 (-1 ~ +1). 정합도.
//                     |r|이 1에 가까울수록 두 센서가 같은 것을 보고 있음.
//     resid         : k로 보정한 뒤 남는 잔차 RMS (dps). 작을수록 좋음.
//
//   축마다 비슷한 세기로 돌려야 비교가 됩니다. 한 축만 약하게 돌리면 그 축의
//   k가 실제보다 작게 나옵니다. 최소자승 기울기는 입력 쪽에 노이즈가 섞이면
//   항상 0쪽으로 편향되기 때문입니다(regression dilution). 이 경우 보고서가
//   그 축을 '회전약함'으로 표시하고 배율 판정을 보류합니다.
//
// ─────────────────────────────────────────────────────────────────────
// SPI 속도 주의
// ─────────────────────────────────────────────────────────────────────
//   MPU-6500은 레지스터 읽기/쓰기가 최대 1MHz입니다(센서 데이터 버스트만
//   20MHz 허용). ICM-42688-P는 전 구간 24MHz까지 가능합니다.
//   두 기종을 같은 코드로 다루므로 안전하게 1MHz로 고정했습니다.
//
// ─────────────────────────────────────────────────────────────────────
// 테스트 진행 방식
// ─────────────────────────────────────────────────────────────────────
//   정지 구간과 세 축의 회전 구간이 각각 목표 표본 수를 채우면 자동으로
//   테스트가 끝나고 보고서를 출력합니다. 수집 중에는 진행률을 축별로
//   표시하므로, 아직 회전이 부족한 축을 바로 알 수 있습니다.
//
//   시험 순서
//     1) 가만히 둔 채 c 를 눌러 바이어스 측정
//     2) 몇 초 더 가만히 두어 정지 구간 표본을 채움
//     3) 세 축을 각각 회전축으로 삼아 좌우로 비틀어 회전 구간 표본을 채움
//        (직선 왕복이 아니라 회전. 위 [중요] 항목 참조)
//     4) 보고서 자동 출력 -> 종료
//        모터 중심축만 검증해도 되면 f 로 조기 종료 가능
//
//   보고서에는 시험 조건, 개별 센서 성능, 장착 정렬, 두 센서 정합,
//   공통 외란 점검, 종합 판정, 그리고 상위 코드에 그대로 복사해 넣을 수 있는
//   바이어스/축 부호 상수가 들어갑니다.
//
// 시리얼 115200.
//   p : 플로터 모드 전환 (두 센서 같은 축을 나란히 출력)
//   c : 자이로 바이어스 측정 (두 센서를 가만히 둔 상태에서 실행)
//   z : 바이어스 해제 후 테스트 재시작
//   r : 테스트 재시작 (누적값 초기화)
//   h : 도움말

#include <SPI.h>

// ===== 핀 (README 핀 배정표 기준) =====
#define PIN_SPI_MISO 4
#define PIN_SPI_MOSI 5
#define PIN_SPI_SCLK 6

#define PIN_CS_1 7
#define PIN_CS_2 15

// ===== SPI =====
// 값이 깨지면 500000, 100000 순으로 낮춰보세요.
const uint32_t SPI_CLOCK_HZ = 1000000;

// ===== 주기 =====
const unsigned long SAMPLE_PERIOD_MS   = 10;     // 100Hz - 통계 표본 수집
const unsigned long PRINT_PERIOD_MS    = 100;    // 10Hz  - 화면 출력
const unsigned long PROGRESS_PERIOD_MS = 2000;   // 진행률 표시 주기
const int           HEADER_EVERY       = 20;

// ===== 구간 판정 =====
// 자이로 크기가 이 값 미만이면 정지로 봅니다.
// 정지 구간의 노이즈로 부호를 판정하면 매번 결과가 뒤집히므로 반드시 나눕니다.
const float STILL_THRESHOLD_DPS = 1.5f;

// 운동 구간은 축마다 따로 셉니다. 해당 축의 각속도가 이 값을 넘을 때만
// 그 축의 표본으로 인정합니다. 한 축만 회전시켰는데 나머지 축까지 표본으로
// 세면, 신호가 없는 축에서 의미 없는 k와 r이 나와 오판을 만듭니다.
const float AXIS_MOTION_THRESHOLD_DPS = 10.0f;

// ===== 테스트 완료 조건 =====
// 두 구간 모두 목표 표본을 채우면 자동으로 보고서를 쓰고 테스트를 끝냅니다.
// 100Hz로 모으므로 600개 = 각 구간 6초 분량입니다.
const uint32_t TARGET_STILL_SAMPLES  = 600;
const uint32_t TARGET_MOTION_SAMPLES = 600;

// ===== 판정 기준 =====
// 보고서의 종합 판정에 쓰는 문턱값입니다.
const float ACCEL_MAG_TOL   = 0.05f;   // |a|가 1.000 에서 이만큼 벗어나면 경고
const float R_MOVE_GOOD     = 0.98f;   // 이 이상이면 정합 양호
const float R_MOVE_LOOSE    = 0.90f;   // 이 미만이면 불일치로 판정
const float K_TOL           = 0.10f;   // |k|가 1.0 에서 이만큼 벗어나면 배율 경고
const float R_STILL_WARN    = 0.30f;   // 정지 노이즈 상관이 이 이상이면 공통 외란 의심
const float SD_WARN_DPS     = 0.50f;   // 자이로 노이즈가 이 이상이면 경고

// 회전 신호 대 잔차 비(SNR). 이 값보다 낮으면 k 추정을 믿을 수 없습니다.
// 최소자승 기울기는 입력에 노이즈가 섞이면 항상 0쪽으로 편향되므로
// (regression dilution), 약하게 회전시킨 축은 |k|가 실제보다 작게 나옵니다.
const float SNR_WARN = 20.0f;

// ===== 바이어스 측정 =====
const int CALIB_SAMPLES = 500;

// ─────────────────────────────────────────────────────────────────────
// 레지스터
// ─────────────────────────────────────────────────────────────────────
#define REG_WHO_AM_I          0x75    // 두 기종 공통 주소

#define WHOAMI_ICM42688       0x47
#define WHOAMI_MPU6500        0x70

// ICM-42688-P (Bank 0)
#define ICM_DEVICE_CONFIG        0x11
#define ICM_TEMP_DATA1           0x1D    // 여기부터 14바이트: 온도2 + 가속도6 + 자이로6
#define ICM_PWR_MGMT0            0x4E
#define ICM_GYRO_CONFIG0         0x4F
#define ICM_ACCEL_CONFIG0        0x50
#define ICM_GYRO_ACCEL_CONFIG0   0x52    // UI 필터 대역폭

// MPU-6500
#define MPU_SMPLRT_DIV        0x19
#define MPU_CONFIG            0x1A
#define MPU_GYRO_CONFIG       0x1B
#define MPU_ACCEL_CONFIG      0x1C
#define MPU_ACCEL_CONFIG2     0x1D
#define MPU_ACCEL_XOUT_H      0x3B    // 여기부터 14바이트: 가속도6 + 온도2 + 자이로6
#define MPU_USER_CTRL         0x6A
#define MPU_PWR_MGMT_1        0x6B
#define MPU_PWR_MGMT_2        0x6C

// ===== 스케일 =====
// 두 기종 모두 16비트 출력이고, 아래 설정에서 환산 계수가 같습니다.
//   가속도 ±16g      -> 2048 LSB/g
//   자이로 ±2000 dps -> 16.4 LSB/dps
const float ACCEL_SCALE_G  = 1.0f / 2048.0f;
const float GYRO_SCALE_DPS = 1.0f / 16.4f;

// ─────────────────────────────────────────────────────────────────────
// 센서
// ─────────────────────────────────────────────────────────────────────
enum ImuType { IMU_NONE, IMU_ICM42688, IMU_MPU6500 };

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

bool bias_valid = false;

// ─────────────────────────────────────────────────────────────────────
// 통계 누적기
// ─────────────────────────────────────────────────────────────────────
// 두 신호의 관계를 풀기 위해 필요한 합들을 축별로 모읍니다.
// 이 여섯 개만 있으면 평균, 표준편차, 상관계수, 최소자승 기울기가 모두 나옵니다.
struct PairStat {
  uint32_t n;
  double s1, s2;        // Σx, Σy
  double s11, s22, s12; // Σx², Σy², Σxy
};

PairStat still_stat[3];    // 정지 구간, 축별
PairStat move_stat[3];     // 운동 구간, 축별

// 정지 구간 가속도 (장착 정렬 확인용)
uint32_t still_accel_n = 0;
double   still_a1[3] = { 0, 0, 0 };
double   still_a2[3] = { 0, 0, 0 };

void statAdd(PairStat& s, double x, double y) {
  s.n++;
  s.s1  += x;      s.s2  += y;
  s.s11 += x * x;  s.s22 += y * y;  s.s12 += x * y;
}

void statClear(PairStat& s) {
  s.n = 0;
  s.s1 = s.s2 = s.s11 = s.s22 = s.s12 = 0;
}

// 평균을 제거한 분산/공분산 (표본이 2개 미만이면 0)
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

// 피어슨 상관계수. -1 ~ +1.
double corrCoef(const PairStat& s) {
  double vx = varX(s), vy = varY(s);
  if (vx <= 0 || vy <= 0) return 0;
  return covXY(s) / sqrt(vx * vy);
}

// 최소자승 기울기 k. y ≈ k*x + b 의 k.
double lsSlope(const PairStat& s) {
  double vx = varX(s);
  if (vx <= 0) return 0;
  return covXY(s) / vx;
}

// k로 보정한 뒤 남는 잔차 RMS.
double residRms(const PairStat& s) {
  double vx = varX(s), vy = varY(s), cxy = covXY(s);
  if (vx <= 0) return 0;
  double ssres = vy - cxy * cxy / vx;    // 설명되지 않은 분산
  if (ssres < 0) ssres = 0;
  return sqrt(ssres);
}

// 전방 선언. 정의는 아래 상태 변수들이 선언된 뒤에 있습니다.
void restartTest();

// ===== 상태 =====
bool plotter_mode = false;
int  line_count   = 0;

// 테스트 진행 단계. 목표 표본을 채우면 COLLECTING -> DONE 으로 넘어가면서
// 보고서를 한 번 출력하고 표본 수집을 멈춥니다.
enum Phase { PHASE_COLLECTING, PHASE_DONE };
Phase phase = PHASE_COLLECTING;

unsigned long test_start_ms    = 0;
unsigned long test_end_ms      = 0;
unsigned long last_sample_ms   = 0;
unsigned long last_print_ms    = 0;
unsigned long last_progress_ms = 0;

// 누적값을 비우고 테스트를 처음부터 다시 시작합니다.
void restartTest() {
  for (int i = 0; i < 3; i++) { statClear(still_stat[i]); statClear(move_stat[i]); }
  still_accel_n = 0;
  for (int i = 0; i < 3; i++) { still_a1[i] = 0; still_a2[i] = 0; }

  phase = PHASE_COLLECTING;
  test_start_ms = millis();
  test_end_ms   = 0;
  line_count    = HEADER_EVERY;
}

// 화면 출력용으로 마지막 표본을 보관합니다.
float last_a[2][3], last_g[2][3], last_t[2];

// ─────────────────────────────────────────────────────────────────────
// SPI 기본 입출력
// ─────────────────────────────────────────────────────────────────────
// 두 기종 모두 읽기는 주소 최상위 비트를 1로 세우고 바로 데이터가 나옵니다.
// 더미 바이트가 없습니다.
uint8_t readReg(uint8_t cs, uint8_t reg) {
  SPI.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  return val;
}

void readRegs(uint8_t cs, uint8_t reg, uint8_t* buf, uint8_t len) {
  SPI.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
  SPI.transfer(reg | 0x80);
  for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

void writeReg(uint8_t cs, uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

// 쓴 값이 실제로 들어갔는지 확인합니다. 배선이 반쯤 되어 있으면
// 읽기는 되는데 쓰기가 안 되는 경우가 있어 여기서 잡힙니다.
void writeRegVerify(uint8_t cs, uint8_t reg, uint8_t val, const char* label) {
  writeReg(cs, reg, val);
  delay(2);
  uint8_t back = readReg(cs, reg);
  if (back != val) {
    char buf[110];
    snprintf(buf, sizeof(buf), "    [WARN] %s (0x%02X): 쓴 값 0x%02X, 읽은 값 0x%02X",
             label, reg, val, back);
    Serial.println(buf);
  }
}

// ─────────────────────────────────────────────────────────────────────
// 센서 판별 / 초기화
// ─────────────────────────────────────────────────────────────────────
ImuType detectSensor(Imu& s) {
  // 전원 인가 직후 첫 트랜잭션이 불안정할 수 있어 몇 번 읽습니다.
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
  writeReg(cs, ICM_DEVICE_CONFIG, 0x01);          // 소프트 리셋
  delay(10);

  writeRegVerify(cs, ICM_GYRO_CONFIG0,  0x06, "GYRO_CONFIG0");   // ±2000 dps, 1kHz
  writeRegVerify(cs, ICM_ACCEL_CONFIG0, 0x06, "ACCEL_CONFIG0");  // ±16g, 1kHz

  // UI 필터 대역폭을 ODR/40 = 25Hz로 좁힙니다. 기본값은 ODR/4 = 250Hz입니다.
  //
  // 이 코드는 100Hz로 읽으므로 나이퀴스트가 50Hz입니다. 기본 250Hz 대역으로
  // 두면 50Hz를 넘는 진동과 손떨림이 접혀 들어옵니다(앨리어싱). 두 센서의
  // 내부 클럭이 서로 독립이라 접히는 모양도 서로 달라, 실제로는 같은 것을
  // 보고 있는데도 상관되지 않는 잔차로 남습니다. 정지 시 노이즈가 0.1 dps인데
  // 회전 중 잔차만 3~5 dps로 커지는 현상이 이것 때문입니다.
  //
  // 25Hz는 밸런싱 제어에서 쓰는 대역보다 충분히 넓어 신호 손실은 없습니다.
  writeRegVerify(cs, ICM_GYRO_ACCEL_CONFIG0, 0x77, "GYRO_ACCEL_CONFIG0");

  // 자이로/가속도 모두 Low Noise 모드로 기동
  // 데이터시트: PWR_MGMT0을 쓴 뒤 200us 동안 다른 레지스터를 건드리지 말 것
  writeReg(cs, ICM_PWR_MGMT0, 0x0F);
  delay(1);
  delay(50);                                       // 자이로 기동 대기
}

void initMPU6500(uint8_t cs) {
  writeReg(cs, MPU_PWR_MGMT_1, 0x80);             // 소프트 리셋
  delay(100);
  writeReg(cs, MPU_PWR_MGMT_1, 0x01);             // 클럭 소스 PLL
  delay(10);

  // I2C 인터페이스를 끄고 SPI 전용으로 고정합니다.
  // 이걸 하지 않으면 SPI 통신 중 I2C 슬레이브가 함께 반응해 값이 깨집니다.
  writeRegVerify(cs, MPU_USER_CTRL,     0x10, "USER_CTRL");
  writeRegVerify(cs, MPU_PWR_MGMT_2,    0x00, "PWR_MGMT_2");
  // DLPF 20Hz. 100Hz로 읽으므로 나이퀴스트가 50Hz입니다. 그보다 넓게 두면
  // 진동이 접혀 들어와(앨리어싱) 두 센서 사이에 상관되지 않는 잔차로 남습니다.
  writeRegVerify(cs, MPU_CONFIG,        0x04, "CONFIG");         // DLPF 20Hz
  writeRegVerify(cs, MPU_SMPLRT_DIV,    0x00, "SMPLRT_DIV");     // 1kHz
  writeRegVerify(cs, MPU_GYRO_CONFIG,   0x18, "GYRO_CONFIG");    // ±2000 dps
  writeRegVerify(cs, MPU_ACCEL_CONFIG,  0x18, "ACCEL_CONFIG");   // ±16g
  writeRegVerify(cs, MPU_ACCEL_CONFIG2, 0x04, "ACCEL_CONFIG2");  // DLPF 20Hz
  delay(50);
}

// ─────────────────────────────────────────────────────────────────────
// 데이터 읽기
// ─────────────────────────────────────────────────────────────────────
static inline int16_t be16(const uint8_t* p) {
  return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

// 가속도는 g, 자이로는 dps로 정규화해서 돌려줍니다.
// 기종이 달라도 위 단위로 통일되므로 아래 분석 코드는 기종을 몰라도 됩니다.
void readSensor(const Imu& s, float a[3], float g[3], float* temp_c) {
  uint8_t buf[14];

  if (s.type == IMU_ICM42688) {
    // 0x1D부터: 온도2 + 가속도6 + 자이로6
    readRegs(s.cs_pin, ICM_TEMP_DATA1, buf, 14);
    *temp_c = (float)be16(&buf[0]) / 132.48f + 25.0f;
    for (int i = 0; i < 3; i++) {
      a[i] = (float)be16(&buf[2 + i * 2]) * ACCEL_SCALE_G;
      g[i] = (float)be16(&buf[8 + i * 2]) * GYRO_SCALE_DPS;
    }
  } else {
    // 0x3B부터: 가속도6 + 온도2 + 자이로6
    readRegs(s.cs_pin, MPU_ACCEL_XOUT_H, buf, 14);
    *temp_c = (float)be16(&buf[6]) / 333.87f + 21.0f;
    for (int i = 0; i < 3; i++) {
      a[i] = (float)be16(&buf[0 + i * 2]) * ACCEL_SCALE_G;
      g[i] = (float)be16(&buf[8 + i * 2]) * GYRO_SCALE_DPS;
    }
  }

  if (bias_valid) {
    for (int i = 0; i < 3; i++) g[i] -= s.bias[i];
  }
}

// ─────────────────────────────────────────────────────────────────────
// 자이로 바이어스 측정
// ─────────────────────────────────────────────────────────────────────
void calibrateGyro() {
  Serial.println();
  Serial.println(F("  자이로 바이어스 측정 중. 두 센서를 움직이지 마세요."));

  bias_valid = false;                    // 원시값으로 측정

  double sum[2][3] = { { 0, 0, 0 }, { 0, 0, 0 } };
  float a[3], g[3], t;

  for (int n = 0; n < CALIB_SAMPLES; n++) {
    for (int s = 0; s < 2; s++) {
      readSensor(imu[s], a, g, &t);
      for (int i = 0; i < 3; i++) sum[s][i] += g[i];
    }
    delay(2);
    if (n % 100 == 0) Serial.print('.');
  }
  Serial.println();

  for (int s = 0; s < 2; s++)
    for (int i = 0; i < 3; i++)
      imu[s].bias[i] = (float)(sum[s][i] / CALIB_SAMPLES);
  bias_valid = true;

  char buf[130];
  for (int s = 0; s < 2; s++) {
    snprintf(buf, sizeof(buf), "  %s 바이어스: X %.3f  Y %.3f  Z %.3f dps",
             imu[s].label, imu[s].bias[0], imu[s].bias[1], imu[s].bias[2]);
    Serial.println(buf);
  }
  Serial.println(F("  이후 출력값에서 자동으로 빼집니다. z 를 누르면 해제됩니다."));
  Serial.println(F("  바이어스가 바뀌었으므로 테스트를 처음부터 다시 시작합니다."));
  restartTest();
  Serial.println();
}

// ─────────────────────────────────────────────────────────────────────
// 표본 수집
// ─────────────────────────────────────────────────────────────────────
void collectSample(const float g1[3], const float g2[3],
                   const float a1[3], const float a2[3]) {
  // 정지 판정은 센서 1의 자이로 전체 크기로 합니다.
  float mag = fabsf(g1[0]) + fabsf(g1[1]) + fabsf(g1[2]);

  if (mag < STILL_THRESHOLD_DPS) {
    for (int i = 0; i < 3; i++) statAdd(still_stat[i], g1[i], g2[i]);

    // 정지일 때만 가속도를 모읍니다. 움직이면 중력 외 성분이 섞입니다.
    still_accel_n++;
    for (int i = 0; i < 3; i++) { still_a1[i] += a1[i]; still_a2[i] += a2[i]; }
    return;
  }

  // 운동 판정은 축마다 따로 합니다. 그 축이 실제로 회전하고 있을 때만
  // 그 축의 표본으로 셉니다. 신호가 없는 축에 표본을 쌓으면 k와 r이
  // 노이즈로 채워져 "불일치"라는 잘못된 판정이 나옵니다.
  for (int i = 0; i < 3; i++) {
    if (fabsf(g1[i]) > AXIS_MOTION_THRESHOLD_DPS)
      statAdd(move_stat[i], g1[i], g2[i]);
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
  Serial.println(F("---- 명령 ----"));
  Serial.println(F("  p : 플로터 모드 전환 (두 센서 같은 축을 나란히 출력)"));
  Serial.println(F("  c : 자이로 바이어스 측정 (정지 상태에서)"));
  Serial.println(F("  z : 바이어스 해제 후 테스트 재시작"));
  Serial.println(F("  r : 테스트 재시작 (누적값 초기화)"));
  Serial.println(F("  f : 조기 종료. 지금까지 모인 표본으로 보고서 작성"));
  Serial.println(F("  h : 도움말"));
}

void printHeader() {
  Serial.println();
  Serial.println(F("        IMU_1 accel (g)         IMU_1 gyro (dps)     |       IMU_2 accel (g)         IMU_2 gyro (dps)"));
  Serial.println(F("      X      Y      Z        X       Y       Z       |     X      Y      Z        X       Y       Z"));
  Serial.println(F("  ----------------------------------------------     +     ----------------------------------------------"));
}

void printDetectFailure(const Imu& s) {
  char buf[110];
  snprintf(buf, sizeof(buf), "  %s WHO_AM_I(0x75) = 0x%02X  (기대: 0x47 ICM-42688-P / 0x70 MPU-6500)",
           s.label, s.whoami);
  Serial.println(buf);

  if (s.whoami == 0xFF) {
    Serial.println(F("    0xFF - MISO가 아무것도 잡지 않고 떠 있습니다."));
    Serial.print  (F("    이 센서의 CS(GPIO")); Serial.print(s.cs_pin);
    Serial.println(F(") 연결과 SDO -> GPIO4 연결을 확인하세요."));
  } else if (s.whoami == 0x00) {
    Serial.println(F("    0x00 - MISO가 GND에 붙어 있거나 센서에 전원이 없습니다."));
  } else {
    Serial.println(F("    아는 ID가 아닙니다. MOSI(GPIO5)/MISO(GPIO4)가 바뀌지 않았는지,"));
    Serial.println(F("    SCK가 GPIO6인지 확인하고 SPI_CLOCK_HZ를 낮춰 재시도하세요."));
  }
}

// 두 벡터 사이 각도 (deg)
float vecAngleDeg(const double u[3], const double v[3]) {
  double du = sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
  double dv = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  if (du <= 0 || dv <= 0) return 0;
  double c = (u[0]*v[0] + u[1]*v[1] + u[2]*v[2]) / (du * dv);
  if (c >  1) c =  1;
  if (c < -1) c = -1;
  return (float)(acos(c) * 180.0 / PI);
}

// ─────────────────────────────────────────────────────────────────────
// 진행률 (수집 중에만)
// ─────────────────────────────────────────────────────────────────────
bool stillDone() { return still_stat[0].n >= TARGET_STILL_SAMPLES; }
bool axisDone(int i) { return move_stat[i].n >= TARGET_MOTION_SAMPLES; }
bool moveDone() { return axisDone(0) && axisDone(1) && axisDone(2); }

void printProgress() {
  char buf[180];
  const char* axis_name[3] = { "X", "Y", "Z" };

  uint32_t sn = still_stat[0].n;
  if (sn > TARGET_STILL_SAMPLES) sn = TARGET_STILL_SAMPLES;

  snprintf(buf, sizeof(buf), "  [수집중] 경과 %.0f초   정지 %lu/%lu (%d%%)",
           (millis() - test_start_ms) / 1000.0f,
           (unsigned long)sn, (unsigned long)TARGET_STILL_SAMPLES,
           (int)(100.0f * sn / TARGET_STILL_SAMPLES));
  Serial.println(buf);

  Serial.print(F("           회전 "));
  for (int i = 0; i < 3; i++) {
    uint32_t mn = move_stat[i].n;
    if (mn > TARGET_MOTION_SAMPLES) mn = TARGET_MOTION_SAMPLES;
    snprintf(buf, sizeof(buf), " %s %lu/%lu(%d%%)", axis_name[i],
             (unsigned long)mn, (unsigned long)TARGET_MOTION_SAMPLES,
             (int)(100.0f * mn / TARGET_MOTION_SAMPLES));
    Serial.print(buf);
  }
  Serial.println();

  // 지금 무엇을 해야 하는지 한 줄로 알려줍니다.
  if (!stillDone()) {
    Serial.println(F("           -> 조립체를 가만히 두세요."));
  } else if (!moveDone()) {
    Serial.print(F("           -> 아직 회전이 부족한 축: "));
    for (int i = 0; i < 3; i++) if (!axisDone(i)) { Serial.print(axis_name[i]); Serial.print(' '); }
    Serial.println();
    Serial.println(F("              그 축을 회전축으로 삼아 좌우로 비트세요(왕복)."));
    Serial.println(F("              축을 따라 밀고 당기는 직선 운동은 자이로에 잡히지 않습니다."));
    Serial.println(F("              모터 중심축만 검증해도 되면 f 로 조기 종료하세요."));
  }

  line_count = HEADER_EVERY;
}

// ─────────────────────────────────────────────────────────────────────
// 보고서
// ─────────────────────────────────────────────────────────────────────
void printRule() {
  Serial.println(F("--------------------------------------------------------------"));
}

void printReport() {
  char buf[190];
  const char* axis_name[3] = { "X", "Y", "Z" };

  // 종합 판정을 위해 경고/실패를 세어 둡니다.
  int n_fail = 0, n_warn = 0;

  Serial.println();
  Serial.println(F("=============================================================="));
  Serial.println(F("                  IMU 정합 테스트 보고서"));
  Serial.println(F("=============================================================="));

  // ---- 시험 조건 ----
  snprintf(buf, sizeof(buf), " 센서 1 : %s  (CS=GPIO%d)", typeName(imu[0].type), imu[0].cs_pin);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), " 센서 2 : %s  (CS=GPIO%d)", typeName(imu[1].type), imu[1].cs_pin);
  Serial.println(buf);
  Serial.println(F(" 설정   : 자이로 ±2000 dps, 가속도 ±16g, 센서 ODR 1kHz, 표본 100Hz"));
  snprintf(buf, sizeof(buf), " SPI    : %.2f MHz, Mode 0", SPI_CLOCK_HZ / 1000000.0f);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), " 표본   : 정지 %lu개,  회전 X %lu / Y %lu / Z %lu 개",
           (unsigned long)still_stat[0].n, (unsigned long)move_stat[0].n,
           (unsigned long)move_stat[1].n,  (unsigned long)move_stat[2].n);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), " 소요   : %.1f 초", (test_end_ms - test_start_ms) / 1000.0f);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), " 바이어스 보정: %s", bias_valid ? "적용됨" : "미적용");
  Serial.println(buf);

  // ---- 1. 개별 센서 성능 ----
  printRule();
  Serial.println(F(" 1. 개별 센서 성능 (정지 구간)"));
  printRule();
  Serial.println(F("   축     bias1      bias2       sd1       sd2"));
  for (int i = 0; i < 3; i++) {
    const PairStat& s = still_stat[i];
    double m1 = s.s1 / s.n, m2 = s.s2 / s.n;
    double d1 = sqrt(varX(s)), d2 = sqrt(varY(s));
    snprintf(buf, sizeof(buf), "   %s   %+9.3f  %+9.3f   %8.3f  %8.3f",
             axis_name[i], m1, m2, d1, d2);
    Serial.println(buf);

    if (d1 > SD_WARN_DPS || d2 > SD_WARN_DPS) n_warn++;
  }
  Serial.println(F("   단위 dps. bias = 자이로 평균(적분 드리프트의 원인),"));
  Serial.println(F("   sd = 자이로 표준편차(노이즈 크기). 둘 다 작을수록 좋습니다."));

  double am1[3] = { 0, 0, 0 }, am2[3] = { 0, 0, 0 };
  double n1 = 0, n2 = 0, angle = 0;
  if (still_accel_n > 0) {
    for (int i = 0; i < 3; i++) {
      am1[i] = still_a1[i] / still_accel_n;
      am2[i] = still_a2[i] / still_accel_n;
    }
    n1 = sqrt(am1[0]*am1[0] + am1[1]*am1[1] + am1[2]*am1[2]);
    n2 = sqrt(am2[0]*am2[0] + am2[1]*am2[1] + am2[2]*am2[2]);
    angle = vecAngleDeg(am1, am2);

    Serial.println();
    snprintf(buf, sizeof(buf), "   가속도 벡터 크기: |a1| %.3f g,  |a2| %.3f g   (기준 1.000 g)", n1, n2);
    Serial.println(buf);
    if (fabs(n1 - 1.0) > ACCEL_MAG_TOL || fabs(n2 - 1.0) > ACCEL_MAG_TOL) {
      Serial.println(F("   [WARN] 1.000 g에서 벗어났습니다. 가속도 스케일 오차 또는 정지 상태가"));
      Serial.println(F("          아니었을 가능성. 스케일 오차면 기울기 추정에 직접 영향을 줍니다."));
      n_warn++;
    }
  }

  // ---- 2. 장착 정렬 ----
  printRule();
  Serial.println(F(" 2. 장착 정렬 (정지 구간 중력 벡터)"));
  printRule();
  if (still_accel_n > 0) {
    snprintf(buf, sizeof(buf), "   두 가속도 벡터 사이 각도: %.1f deg", angle);
    Serial.println(buf);
    if (angle < 15.0)
      Serial.println(F("   해석: 두 센서가 같은 방향으로 장착되었습니다."));
    else if (angle > 165.0)
      Serial.println(F("   해석: 두 센서가 서로 뒤집혀 장착되었습니다(앞뒤 배치). 정상입니다."));
    else {
      Serial.println(F("   [WARN] 0도도 180도도 아닙니다. 두 센서가 서로 기울어져 붙어 있습니다."));
      Serial.println(F("          같은 축 가정이 깨지므로 장착을 다시 확인하세요."));
      n_warn++;
    }
  } else {
    Serial.println(F("   정지 구간 가속도 표본이 없습니다."));
  }

  // ---- 3. 두 센서 정합 ----
  printRule();
  Serial.println(F(" 3. 두 센서 정합 (회전 구간)"));
  printRule();
  Serial.println(F("   축   표본   회전크기   SNR   부호관계      k         r_move     resid   판정"));

  float axis_sign[3] = { 1, 1, 1 };
  bool  axis_valid[3] = { false, false, false };
  int   n_unjudged = 0;
  int   n_lowsnr   = 0;

  for (int i = 0; i < 3; i++) {
    const PairStat& s = move_stat[i];

    // 그 축을 회전시키지 않았으면 판정 자체가 불가능합니다.
    // 신호가 없는 데이터로 계산한 k와 r은 노이즈일 뿐이라 출력하지 않습니다.
    if (s.n < TARGET_MOTION_SAMPLES) {
      snprintf(buf, sizeof(buf), "   %s  %5lu       -       -    %s",
               axis_name[i], (unsigned long)s.n,
               "-          -          -          -   판정불가(회전 부족)");
      Serial.println(buf);
      n_unjudged++;
      continue;
    }

    double k = lsSlope(s);
    double r = corrCoef(s);
    double e = residRms(s);
    double ak = fabs(k);

    // 이 축에서 실제로 얼마나 크게 회전시켰는지. 잔차 대비 비가 SNR입니다.
    double amp = sqrt(varX(s));
    double snr = (e > 0) ? (amp / e) : 0;

    axis_sign[i]  = (k >= 0) ? 1.0f : -1.0f;
    axis_valid[i] = true;
    const char* sign = (k >= 0) ? "S2 = +S1" : "S2 = -S1";

    const char* verdict;
    if (snr < SNR_WARN) {
      // SNR이 낮으면 k가 0쪽으로 편향되므로 배율 판정을 하지 않습니다.
      // 부호와 상관계수는 여전히 유효합니다.
      verdict = "회전약함";
      n_lowsnr++;
      n_warn++;
    } else if (fabs(r) > R_MOVE_GOOD && ak > (1.0f - K_TOL) && ak < (1.0f + K_TOL)) {
      verdict = "일치";
    } else if (fabs(r) > R_MOVE_LOOSE) {
      verdict = "느슨";
      n_warn++;
    } else {
      verdict = "불일치";
      n_fail++;
    }

    snprintf(buf, sizeof(buf), "   %s  %5lu   %8.1f  %6.1f  %s  %+9.4f  %+9.4f  %7.3f   %s",
             axis_name[i], (unsigned long)s.n, amp, snr, sign, k, r, e, verdict);
    Serial.println(buf);
  }
  Serial.println(F("   회전크기 = 그 축 각속도의 표준편차 (dps). 얼마나 세게 돌렸는지."));
  Serial.println(F("   SNR = 회전크기 / resid. 클수록 k를 믿을 수 있습니다."));
  Serial.println(F("   k = 최소자승 기울기 (g2 ≈ k*g1). 부호가 축 관계, 크기가 배율 차이."));
  Serial.println(F("   r_move = 피어슨 상관계수. 1에 가까울수록 같은 것을 보고 있음."));
  Serial.println(F("   resid = k로 보정한 뒤 남는 잔차 RMS (dps)."));

  if (n_lowsnr > 0) {
    Serial.println();
    Serial.println(F("   ['회전약함' 축에 대하여]"));
    Serial.print  (F("   SNR이 "));
    Serial.print(SNR_WARN, 0);
    Serial.println(F(" 미만입니다. 그 축을 다른 축보다 약하게 돌렸다는 뜻입니다."));
    Serial.println(F("   최소자승 기울기는 입력 쪽에 노이즈가 섞이면 항상 0쪽으로 편향됩니다."));
    Serial.println(F("   (regression dilution) 따라서 실제 |k|는 표에 찍힌 값보다 1에 가깝습니다."));
    Serial.println(F("   부호관계와 r_move는 그대로 신뢰해도 됩니다. 배율만 믿지 마세요."));
    Serial.println(F("   그 축을 다른 축과 비슷한 세기로 다시 돌려 재측정하세요."));
  }

  if (n_unjudged > 0) {
    Serial.println();
    Serial.println(F("   [판정불가 축에 대하여]"));
    Serial.println(F("   그 축을 회전축으로 삼는 회전을 주지 않아 신호가 없었습니다."));
    Serial.println(F("   자이로는 회전만 읽습니다. 축을 따라 밀고 당기는 직선 운동으로는"));
    Serial.println(F("   표본이 쌓이지 않습니다. 해당 축을 중심으로 좌우로 비트세요."));
    Serial.println(F("   밸런싱에 쓰는 모터 중심축만 검증하는 것이 목적이라면 무시해도 됩니다."));
  }

  // ---- 4. 공통 외란 점검 ----
  printRule();
  Serial.println(F(" 4. 공통 외란 점검 (정지 구간 노이즈 상관)"));
  printRule();
  bool common_noise = false;
  Serial.print(F("   r_still :"));
  for (int i = 0; i < 3; i++) {
    double r = corrCoef(still_stat[i]);
    snprintf(buf, sizeof(buf), "  %s %+.3f", axis_name[i], r);
    Serial.print(buf);
    if (fabs(r) > R_STILL_WARN) common_noise = true;
  }
  Serial.println();
  if (common_noise) {
    Serial.println(F("   [WARN] 두 센서의 노이즈가 함께 움직입니다. 서로 독립이어야 정상입니다."));
    Serial.println(F("          전원 공용 임피던스나 기구 진동을 함께 타고 있을 가능성."));
    Serial.println(F("          센서를 2개 쓰는 이점(평균화로 노이즈 감소)이 줄어듭니다."));
    n_warn++;
  } else {
    Serial.println(F("   정상. 두 센서의 노이즈가 서로 독립입니다. 평균화 효과를 기대할 수 있습니다."));
  }

  // ---- 5. 종합 판정 ----
  printRule();
  Serial.println(F(" 5. 종합 판정"));
  printRule();
  if (n_fail > 0) {
    Serial.println(F("   [FAIL] 두 센서가 같은 것을 보고 있다고 보기 어렵습니다."));
    Serial.println(F("          축 정합이 깨졌거나 한쪽 센서의 데이터가 잘못되고 있습니다."));
    Serial.println(F("          확인: 장착 방향, CS 배선, SPI 클럭, 두 센서가 같은 강체에 붙었는지."));
  } else if (n_warn > 0) {
    snprintf(buf, sizeof(buf), "   [WARN] 기본 정합은 성립하나 경고 %d건이 있습니다. 위 항목을 확인하세요.", n_warn);
    Serial.println(buf);
  } else {
    Serial.println(F("   [PASS] 두 센서가 같은 축에서 정상적으로 정합됩니다."));
  }

  // ---- 6. 상위 코드에 반영할 값 ----
  printRule();
  Serial.println(F(" 6. 상위 코드에 반영할 값 (복사해서 사용)"));
  printRule();
  Serial.println(F("   // 자이로 바이어스 (dps). 읽은 값에서 빼서 사용합니다."));
  for (int s = 0; s < 2; s++) {
    snprintf(buf, sizeof(buf), "   const float GYRO_BIAS_%d[3] = { %+.4ff, %+.4ff, %+.4ff };",
             s + 1, imu[s].bias[0], imu[s].bias[1], imu[s].bias[2]);
    Serial.println(buf);
  }
  Serial.println(F("   // IMU_2를 IMU_1 기준으로 맞추는 축 부호. 곱해서 사용합니다."));
  snprintf(buf, sizeof(buf), "   const float AXIS_SIGN_2[3] = { %+.0ff, %+.0ff, %+.0ff };",
           axis_sign[0], axis_sign[1], axis_sign[2]);
  Serial.println(buf);
  if (n_unjudged > 0) {
    Serial.print(F("   [주의] 판정불가 축("));
    for (int i = 0; i < 3; i++) if (!axis_valid[i]) { Serial.print(axis_name[i]); Serial.print(' '); }
    Serial.println(F(")의 부호는 측정값이 아니라 기본값 +1 입니다."));
    Serial.println(F("          그 축을 실제로 쓰려면 해당 축 회전을 준 뒤 다시 측정하세요."));
  }
  if (!bias_valid) {
    Serial.println(F("   [주의] 바이어스를 측정하지 않아 위 GYRO_BIAS 값이 모두 0입니다."));
    Serial.println(F("          r 로 다시 시작하고 정지 상태에서 c 를 먼저 누르세요."));
  }

  Serial.println(F("=============================================================="));
  Serial.println(F(" 테스트 종료. r = 다시 측정,  p = 플로터 모드,  h = 도움말"));
  Serial.println(F("=============================================================="));
  Serial.println();
}

// ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("=========================================="));
  Serial.println(F(" IMU x2 monitor + 정합 분석 - MAIN (ESP32-S3)"));
  Serial.print  (F(" SPI: SCK=GPIO"));  Serial.print(PIN_SPI_SCLK);
  Serial.print  (F(" MOSI=GPIO"));      Serial.print(PIN_SPI_MOSI);
  Serial.print  (F(" MISO=GPIO"));      Serial.println(PIN_SPI_MISO);
  Serial.print  (F(" CS: IMU_1=GPIO")); Serial.print(PIN_CS_1);
  Serial.print  (F("  IMU_2=GPIO"));    Serial.println(PIN_CS_2);
  Serial.print  (F(" SPI clock: "));    Serial.print(SPI_CLOCK_HZ / 1000000.0f, 2);
  Serial.println(F(" MHz"));
  Serial.println(F("=========================================="));

  // CS를 먼저 전부 HIGH로 올려둡니다.
  // 초기화 중에 다른 칩이 선택된 상태로 남아 있으면 버스에서 충돌합니다.
  for (int s = 0; s < 2; s++) {
    pinMode(imu[s].cs_pin, OUTPUT);
    digitalWrite(imu[s].cs_pin, HIGH);
  }

  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
  delay(10);

  Serial.println(F("센서 판별 중..."));
  int ok = 0;
  for (int s = 0; s < 2; s++) {
    imu[s].type = detectSensor(imu[s]);
    if (imu[s].type == IMU_NONE) {
      printDetectFailure(imu[s]);
    } else {
      char buf[90];
      snprintf(buf, sizeof(buf), "  %s : %s (WHO_AM_I = 0x%02X, CS=GPIO%d)",
               imu[s].label, typeName(imu[s].type), imu[s].whoami, imu[s].cs_pin);
      Serial.println(buf);
      ok++;
    }
  }

  if (ok < 2) {
    Serial.println();
    Serial.println(F("[FAIL] 두 센서가 모두 필요합니다. 정합 분석은 2개가 있어야 성립합니다."));
    if (ok == 1) {
      Serial.println(F("  한쪽만 안 되면 SPI 3선이 아니라 그 센서의 CS 배선 문제입니다."));
      Serial.println(F("  (공통선이 문제면 양쪽 다 실패했을 것이기 때문입니다.)"));
    }
    while (1) delay(500);
  }

  Serial.println(F("초기화 중..."));
  for (int s = 0; s < 2; s++) {
    if (imu[s].type == IMU_ICM42688) initICM42688(imu[s].cs_pin);
    else                             initMPU6500(imu[s].cs_pin);
  }
  Serial.println(F("  완료. 자이로 ±2000 dps, 가속도 ±16g, 1kHz"));

  Serial.println();
  printHelp();
  Serial.println();
  Serial.println(F("--- 시험 순서 ---"));
  Serial.println(F("  1) 조립체를 가만히 둔 채 c 를 눌러 바이어스 측정"));
  Serial.println(F("  2) 몇 초 더 가만히 두어 [정지 구간] 표본을 채움"));
  Serial.println(F("  3) 조립체를 '회전'시켜 [회전 구간] 표본을 채움"));
  Serial.println();
  Serial.println(F("--- 3) 에서 무엇을 해야 하는가 ---"));
  Serial.println(F("  자이로는 회전만 읽습니다. 축을 따라 밀고 당기는 직선 운동은"));
  Serial.println(F("  각속도가 0이라 아무 신호도 만들지 않습니다. 반드시 '비트는' 동작이어야 합니다."));
  Serial.println(F("  두 센서가 모터 중심축 위 다른 위치에 있어도, 각속도는 강체 어디서나"));
  Serial.println(F("  같으므로 위치 차이는 문제가 되지 않습니다."));
  Serial.println();
  Serial.println(F("  세 축을 각각 회전축으로 삼아 좌우 왕복으로 비트세요:"));
  Serial.println(F("    - 모터 중심축을 회전축으로 비틀기   <- 밸런싱 제어에 쓰는 축"));
  Serial.println(F("    - 그 축에 수직인 방향으로 기울이기 (앞뒤로 끄덕이듯)"));
  Serial.println(F("    - 나머지 한 방향으로 기울이기      (좌우로 갸웃하듯)"));
  Serial.println();
  Serial.print  (F("  정지 "));
  Serial.print(TARGET_STILL_SAMPLES);
  Serial.print  (F("개, 회전은 축마다 "));
  Serial.print(TARGET_MOTION_SAMPLES);
  Serial.println(F("개를 채우면 자동으로 보고서가 출력됩니다."));
  Serial.println(F("  모터 중심축만 검증해도 되면 f 로 조기 종료하세요."));
  Serial.println();

  unsigned long now = millis();
  last_sample_ms   = now;
  last_print_ms    = now;
  last_progress_ms = now;
  restartTest();
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'p':
        plotter_mode = !plotter_mode;
        line_count = HEADER_EVERY;
        Serial.print(F("-> 플로터 모드 "));
        Serial.println(plotter_mode ? F("ON") : F("OFF"));
        break;
      case 'c':
        calibrateGyro();
        break;
      case 'z':
        bias_valid = false;
        for (int s = 0; s < 2; s++)
          for (int i = 0; i < 3; i++) imu[s].bias[i] = 0;
        Serial.println(F("-> 바이어스 해제. 테스트를 처음부터 다시 시작합니다."));
        restartTest();
        break;
      case 'r':
        Serial.println(F("-> 테스트를 처음부터 다시 시작합니다."));
        restartTest();
        break;
      case 'f':
        // 모터 중심축 하나만 검증하면 되는 경우를 위한 조기 종료.
        // 표본이 모자란 축은 보고서에서 판정불가로 표시됩니다.
        if (phase == PHASE_COLLECTING) {
          Serial.println(F("-> 조기 종료. 현재까지 모인 표본으로 보고서를 작성합니다."));
          phase       = PHASE_DONE;
          test_end_ms = millis();
          printReport();
        } else {
          Serial.println(F("-> 이미 종료된 상태입니다. r 로 다시 시작하세요."));
        }
        break;
      case 'h':
        printHelp();
        break;
    }
  }
}

void loop() {
  handleSerial();

  unsigned long now = millis();

  // ---- 표본 수집 (100Hz) ----
  // 통계 품질은 표본 수가 좌우하므로 화면 출력보다 빠르게 돕니다.
  if (now - last_sample_ms >= SAMPLE_PERIOD_MS) {
    last_sample_ms = now;

    for (int s = 0; s < 2; s++)
      readSensor(imu[s], last_a[s], last_g[s], &last_t[s]);

    // 테스트가 끝난 뒤에는 화면만 갱신하고 통계는 더 쌓지 않습니다.
    if (phase == PHASE_COLLECTING) {
      collectSample(last_g[0], last_g[1], last_a[0], last_a[1]);

      // 정지 구간과 세 축 회전이 모두 목표를 채우면 보고서를 쓰고 종료합니다.
      if (stillDone() && moveDone()) {
        phase       = PHASE_DONE;
        test_end_ms = now;
        printReport();
      }
    }
  }

  // ---- 화면 출력 (10Hz) ----
  if (now - last_print_ms < PRINT_PERIOD_MS) return;
  last_print_ms = now;

  // 테스트가 끝나면 표를 계속 흘리지 않습니다. 보고서가 화면에 남아 있어야
  // 읽을 수 있기 때문입니다. 플로터 모드만 계속 동작합니다.
  if (phase == PHASE_DONE && !plotter_mode) return;

  if (plotter_mode) {
    // Arduino 시리얼 플로터용. 숫자와 탭만 출력해야 그래프가 그려집니다.
    // 두 센서의 같은 축을 나란히 두어 부호 관계가 눈으로 보이게 배치했습니다.
    Serial.printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
                  last_g[0][0], last_g[1][0],
                  last_g[0][1], last_g[1][1],
                  last_g[0][2], last_g[1][2]);
    return;
  }

  if (line_count >= HEADER_EVERY) { printHeader(); line_count = 0; }
  line_count++;

  char buf[210];
  snprintf(buf, sizeof(buf),
           "  %6.2f %6.2f %6.2f   %7.1f %7.1f %7.1f   |  %6.2f %6.2f %6.2f   %7.1f %7.1f %7.1f",
           last_a[0][0], last_a[0][1], last_a[0][2],
           last_g[0][0], last_g[0][1], last_g[0][2],
           last_a[1][0], last_a[1][1], last_a[1][2],
           last_g[1][0], last_g[1][1], last_g[1][2]);
  Serial.println(buf);

  if (phase == PHASE_COLLECTING && now - last_progress_ms >= PROGRESS_PERIOD_MS) {
    last_progress_ms = now;
    printProgress();
  }
}
