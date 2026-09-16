// Nidec 24H055M020 / ESP32-C3 hardware test
// Reference: ../04_nidec_instructables_port/04_nidec_instructables_port.ino
// Wiring: ../../Nidec_ESP32C3_테스트.md (connector numbering follows that file).
#include <Arduino.h>
#include <esp_arduino_version.h>
#include <stdlib.h>
#include <string.h>

#if !defined(CONFIG_IDF_TARGET_ESP32C3)
#error "Select an ESP32-C3 board."
#endif

constexpr uint8_t PFM_PIN = 4, ENABLE_PIN = 5, DIR_PIN = 6;
constexpr uint8_t BRAKE_PIN = 7, FG_PIN = 10;
constexpr uint8_t PWM_BITS = 10, PWM_CHANNEL = 0;
constexpr uint32_t PWM_DUTY = 819; // 819 / 1024 = approximately 80%
constexpr uint32_t MIN_HZ = 250, MAX_HZ = 26000;
uint32_t requestedHz = 1000, actualHz = 0;
uint32_t pulsesPerRevolution = 0; // Unknown: set with "ppr N" after calibration.
bool running = false, pwmAttached = false;
char brakeState = 'Z'; // Z=input, L=LOW, H=HIGH; polarity is not assumed verified.
volatile uint32_t fgEdges = 0;
portMUX_TYPE fgMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t lastReportMs = 0;
char line[64];
size_t lineLength = 0;
bool lineOverflow = false;

void ARDUINO_ISR_ATTR countFg() {
  portENTER_CRITICAL_ISR(&fgMux);
  ++fgEdges;
  portEXIT_CRITICAL_ISR(&fgMux);
}

void stopMotor() {
  digitalWrite(ENABLE_PIN, LOW);
  running = false;
  if (pwmAttached) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcDetach(PFM_PIN);
#else
    ledcDetachPin(PFM_PIN);
#endif
    pwmAttached = false;
  }
  pinMode(PFM_PIN, OUTPUT);
  digitalWrite(PFM_PIN, LOW);
  actualHz = 0;
}

bool applyPfm() {
  // Disable while reconfiguring; enable only after successful PWM setup.
  stopMotor();
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!ledcAttach(PFM_PIN, requestedHz, PWM_BITS)) {
    Serial.println("ERR: ledcAttach failed (GPIO4, 10-bit)");
    stopMotor();
    return false;
  }
  pwmAttached = true;
  if (!ledcWrite(PFM_PIN, PWM_DUTY)) {
    Serial.println("ERR: ledcWrite failed (duty=819)");
    stopMotor();
    return false;
  }
  // Duty updates become visible at a PWM cycle boundary. Arduino core 3.x
  // ledcReadFreq returns 0 while the hardware duty still reads as 0.
  // Keep Enable LOW and allow up to 20 ms (5 cycles at the minimum 250 Hz).
  for (uint8_t attempt = 0; attempt < 20; ++attempt) {
    delay(1);
    actualHz = ledcReadFreq(PFM_PIN);
    if (actualHz) break;
  }
#else
  double hz = ledcSetup(PWM_CHANNEL, requestedHz, PWM_BITS);
  if (hz <= 0) {
    Serial.println("ERR: ledcSetup failed");
    return false;
  }
  ledcAttachPin(PFM_PIN, PWM_CHANNEL);
  pwmAttached = true;
  ledcWrite(PWM_CHANNEL, PWM_DUTY);
  actualHz = static_cast<uint32_t>(hz);
#endif
  if (!actualHz) {
    Serial.println("ERR: PWM frequency readback stayed zero after settling");
    stopMotor();
    return false;
  }
  digitalWrite(ENABLE_PIN, HIGH);
  running = true;
  return true;
}

void help() {
  Serial.println("Commands (newline, 115200 baud):");
  Serial.println("run | stop | f 250..26000 | dir 0|1 | ppr 0..100000");
  Serial.println("brake low|high|off | help");
  Serial.println("! = immediate stop; dir/brake changes stop the motor first.");
  Serial.println("Brake starts floating (Z). LOW/HIGH polarity needs verification.");
  Serial.println("ppr 0 = unknown; RPM estimate from PFM is NOT measured RPM.");
}

bool parseNumber(const char *s, uint32_t &value) {
  if (!s || !*s) return false;
  uint32_t n = 0;
  for (; *s; ++s) {
    if (*s < '0' || *s > '9') return false;
    const uint32_t digit = *s - '0';
    if (n > (UINT32_MAX - digit) / 10) return false;
    n = n * 10 + digit;
  }
  value = n;
  return true;
}

void command(char *text) {
  char *cmd = strtok(text, " \t");
  char *arg = strtok(nullptr, " \t");
  char *extra = strtok(nullptr, " \t");
  if (!cmd) return;
  uint32_t value = 0;
  if (extra) { Serial.println("ERR: too many arguments"); return; }
  if (!strcmp(cmd, "stop") && !arg) {
    stopMotor(); Serial.println("STOP: enable LOW, PFM LOW");
  } else if (!strcmp(cmd, "run") && !arg) {
    Serial.println(applyPfm() ? "RUN" : "ERR: PWM setup failed; stopped");
  } else if (!strcmp(cmd, "help") && !arg) {
    help();
  } else if (!strcmp(cmd, "f") && parseNumber(arg, value) &&
             value >= MIN_HZ && value <= MAX_HZ) {
    requestedHz = value;
    if (running && !applyPfm()) Serial.println("ERR: PWM setup failed; stopped");
    Serial.printf("Requested PFM: %lu Hz\n", (unsigned long)requestedHz);
  } else if (!strcmp(cmd, "dir") && parseNumber(arg, value) && value <= 1) {
    stopMotor();
    digitalWrite(DIR_PIN, value ? HIGH : LOW);
    Serial.println("Direction set; wait for complete standstill before run.");
  } else if (!strcmp(cmd, "ppr") && parseNumber(arg, value) && value <= 100000) {
    pulsesPerRevolution = value;
    Serial.printf("FG pulses/revolution: %lu\n", (unsigned long)value);
  } else if (!strcmp(cmd, "brake") && arg &&
             (!strcmp(arg, "low") || !strcmp(arg, "high") || !strcmp(arg, "off"))) {
    stopMotor();
    if (!strcmp(arg, "off")) {
      pinMode(BRAKE_PIN, INPUT);
      brakeState = 'Z';
    } else {
      bool high = !strcmp(arg, "high");
      pinMode(BRAKE_PIN, OUTPUT);
      digitalWrite(BRAKE_PIN, high ? HIGH : LOW);
      brakeState = high ? 'H' : 'L';
    }
    Serial.printf("Brake=%c; enable LOW. Explicit run required.\n", brakeState);
  } else {
    Serial.println("ERR: invalid command/range; type help");
  }
}

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  pinMode(PFM_PIN, OUTPUT);
  digitalWrite(PFM_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH); // Original example: HIGH direction.
  pinMode(BRAKE_PIN, INPUT); // No internal pull-up: unverified brake signal.
  pinMode(FG_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FG_PIN), countFg, FALLING);
  Serial.begin(115200);
  lastReportMs = millis();
  help(); // No waiting for USB, no automatic motor start.
}

void loop() {
  // Bound work per loop so continuous serial input cannot starve telemetry.
  for (uint8_t i = 0; i < 64 && Serial.available(); ++i) {
    char c = Serial.read();
    if (c == '!') {
      stopMotor();
      lineLength = 0;
      lineOverflow = true; // Discard the rest of this line, including queued run.
      Serial.println("STOP!");
    } else if (c == '\n' || c == '\r') {
      if (!lineOverflow) { line[lineLength] = '\0'; command(line); }
      lineLength = 0;
      lineOverflow = false;
    } else if (!lineOverflow) {
      if (lineLength < sizeof(line) - 1) line[lineLength++] = c;
      else { lineOverflow = true; Serial.println("ERR: line too long; discarded"); }
    }
  }

  uint32_t now = millis();
  uint32_t elapsed = now - lastReportMs;
  if (elapsed >= 1000) {
    portENTER_CRITICAL(&fgMux);
    uint32_t count = fgEdges;
    fgEdges = 0;
    portEXIT_CRITICAL(&fgMux);
    lastReportMs = now;
    float fgHz = count * (1000.0f / elapsed);
    Serial.printf("run=%u dir=%d brake=%c set_hz=%lu pfm_hz=%lu fg_count=%lu fg_hz=%.2f rpm_fg=",
                  running, digitalRead(DIR_PIN), brakeState,
                  (unsigned long)requestedHz, (unsigned long)actualHz,
                  (unsigned long)count, fgHz);
    if (pulsesPerRevolution) Serial.print(fgHz * 60.0f / pulsesPerRevolution, 2);
    else Serial.print("NA");
    Serial.printf(" rpm_command_est=%.2f\n", actualHz / 6.6667f);
  }
}
