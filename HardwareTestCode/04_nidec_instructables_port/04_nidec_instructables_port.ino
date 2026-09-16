// Nidec 24H055M020 — Instructables 원본 코드 포팅 (기본 회전 테스트, ESP32-C3)
//
// 원본: Nidec.md 3.2절 / Instructables(NewsonsElectronics) 공개 코드.
// 방침: 원래 코드를 최대한 그대로 유지하고, ESP32-C3에서 불가피한 부분만 바꿈.
//
//   - AVR Timer1 레지스터(TCCR1A/ICR1/OCR1A) → ESP32 LEDC로 교체 (플랫폼이 다름).
//     setPWMFrequency() 함수명·역할은 원본 그대로 유지.
//   - Uno 핀 번호 → Nidec_ESP32C3_테스트.md 4절 GPIO 매핑으로 교체.
//   - Brake 핀은 원본 코드도 아예 건드리지 않음 — 이 스케치도 동일하게 미사용.
//   - Hall 원신호(H1/H2/H3) 평균 코드는 뺐음: 이 배선은 연결 안 했고(FG 개조
//     핀만 사용), ESP32-C3는 ADC 핀이 4개뿐이라 자리도 부족함. "기본 회전"
//     확인에는 불필요.
//   - 포텐셔미터도 뺐음 — 고정 주파수(PFM_FIXED_HZ)로 시작. 값만 바꿔
//     재업로드하면 여러 주파수를 빠르게 시험할 수 있음.
//   - 딱 한 가지 안전장치만 추가: 02번 스케치 디버깅에서 "동일한 값으로
//     ledcChangeFrequency를 계속 재호출하면 회전이 깨진다"는 게 실측으로
//     확인되어, setPWMFrequency()에 "값이 실제로 바뀔 때만 적용" 가드를
//     한 줄 추가함. 원본이 매 루프 이 함수를 부르는 구조 자체는 그대로 둠.

#define PIN_PFM       4   // Yellow, 12번 핀
#define PIN_ENABLE    5   // Blue,   11번 핀
#define PIN_DIRECTION 6   // Green,  9번 핀
#define PIN_FG        10  // Orange, 8번 핀(개조됨)

const int fgPin = PIN_FG; // using interrupt to count pulses on FG pin

// 원본의 pfmFrequency 초기값(4000)과 동일. 이 값만 바꿔서 재업로드하면
// 다른 주파수(예: 10000, 15500)에서의 회전 여부를 바로 시험할 수 있음.
const unsigned long PFM_FIXED_HZ = 4000;

float dutyPercent = 80.0; // 원본과 동일 (Nidec.md 3.2절 실측 성공값, 50% 아님)
float RPM = 0;            // Rounds per Minute = Frequency/6.6667 (원본과 동일 상수)

volatile unsigned long edgeCount = 0;
float frequency = 0.0;

unsigned long previousMillis = 0;
const unsigned long frequencyTime = 1000; // 원본과 동일 (FG 주파수 계산 주기)

const uint8_t PFM_RESOLUTION_BITS = 10;
unsigned long lastAppliedFreq = 0; // ESP32 LEDC 재호출 글리치 방지용 (원본엔 없던 최소 가드)

// ===== FG Interrupt =====
void IRAM_ATTR countFallingEdge() {
  edgeCount++;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_DIRECTION, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);

  digitalWrite(PIN_DIRECTION, HIGH);
  digitalWrite(PIN_ENABLE, HIGH);

  pinMode(fgPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(fgPin), countFallingEdge, FALLING);

  // ===== PFM 출력 (ESP32 LEDC — 원본 Timer1 PWM 자리) =====
  ledcAttach(PIN_PFM, PFM_FIXED_HZ, PFM_RESOLUTION_BITS);
  setPWMFrequency(PFM_FIXED_HZ);
}

void loop() {
  unsigned long now = millis();

  // 원본처럼 매 루프 호출하지만, 고정값이라 항상 같은 값 → 아래 가드 덕분에
  // 실제로는 최초 1회만 적용되고 이후엔 LEDC를 다시 건드리지 않음.
  setPWMFrequency(PFM_FIXED_HZ);
  RPM = PFM_FIXED_HZ / 6.6667;

  // ===== Frequency Calculation (원본과 동일) =====
  if (now - previousMillis >= frequencyTime) {
    previousMillis = now;

    noInterrupts();
    unsigned long count = edgeCount;
    edgeCount = 0;
    interrupts();

    frequency = count * (1000.0 / frequencyTime);

    Serial.print(" PFM:");
    Serial.print(PFM_FIXED_HZ);
    Serial.print("hz");
    Serial.print(" FG:");
    Serial.print(frequency, 0);
    Serial.print(" RPM:");
    Serial.print(RPM, 0);
    Serial.println("");
  }
}

// ===== PFM Function (ESP32 LEDC 버전, 원본 setPWMFrequency 대체) =====
void setPWMFrequency(unsigned long freq) {
  if (freq == 0) freq = 1;
  if (freq == lastAppliedFreq) return;  // 동일 값 재호출로 인한 LEDC 글리치 방지

  ledcChangeFrequency(PIN_PFM, freq, PFM_RESOLUTION_BITS);
  uint32_t dutyCount = (uint32_t)(dutyPercent / 100.0 * ((1UL << PFM_RESOLUTION_BITS) - 1));
  ledcWrite(PIN_PFM, dutyCount);
  lastAppliedFreq = freq;
}
