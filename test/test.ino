#include <SimpleFOC.h>

// 핀 정의
#define IN1_PIN 11
#define IN2_PIN 10
#define IN3_PIN 9
#define EN_PIN 8

// AS5600 자기 센서
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// BLDC 드라이버
BLDCDriver3PWM driver = BLDCDriver3PWM(IN1_PIN, IN2_PIN, IN3_PIN, EN_PIN);

// BLDC 모터
BLDCMotor motor = BLDCMotor(7);

// 센서 테스트
bool testSensor() {
  Serial.println(F("\n[1/4] Sensor test..."));
  float angle = sensor.getAngle();
  Serial.println(angle, 2);
  
  if (angle == 0) {
    Serial.println(F("FAIL"));
    return false;
  }
  Serial.println(F("PASS"));
  return true;
}

// 폴 페어 찾기
int findPolePairs() {
  Serial.println(F("\n[2/4] Finding pole pairs..."));
  
  delay(500);
  float start_angle = sensor.getAngle();
  
  // 전기각 1회전
  for (int i = 0; i <= 314; i++) {
    motor.move(i * 0.02);
    motor.loopFOC();
    delay(15);
  }
  
  delay(500);
  float end_angle = sensor.getAngle();
  
  float angle_change = end_angle - start_angle;
  if (angle_change < 0) angle_change += 2 * PI;
  
  float mechanical_rotations = angle_change / (2 * PI);
  int pole_pairs = round(1.0 / mechanical_rotations);
  
  Serial.print(F("Result: "));
  Serial.println(pole_pairs);
  
  return pole_pairs;
}

// 회전 테스트
void testRotation(int pole_pairs) {
  Serial.println(F("\n[3/4] Rotation test..."));
  
  motor.pole_pairs = pole_pairs;
  
  // 점진적 가속
  float v = 0;
  while (v < 10) {
    v += 0.1;
    motor.move(v);
    motor.loopFOC();
    delay(10);
  }
  
  // 2초 유지
  delay(2000);
  
  // 감속
  while (v > 0) {
    v -= 0.1;
    motor.move(v);
    motor.loopFOC();
    delay(10);
  }
  
  Serial.println(F("PASS"));
}

// 최종 확인
void finalTest(int pp) {
  Serial.println(F("\n[4/4] Final test..."));
  
  float v = 0;
  
  // 가속
  while (v < 15) {
    v += 0.15;
    motor.move(v);
    motor.loopFOC();
    delay(10);
  }
  
  // 유지
  delay(2000);
  
  // 감속
  while (v > 0) {
    v -= 0.15;
    if (v < 0) v = 0;
    motor.move(v);
    motor.loopFOC();
    delay(10);
  }
  
  Serial.println(F("COMPLETE"));
  Serial.print(F("Pole pairs: "));
  Serial.println(pp);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\nMotor Test"));
  
  Wire.begin();
  sensor.init();
  
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  
  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = 2;
  motor.init();
  
  delay(500);
  
  if (!testSensor()) {
    Serial.println(F("ERROR"));
    while(1);
  }
  
  int pp = findPolePairs();
  motor.pole_pairs = pp;
  
  testRotation(pp);
  finalTest(pp);
  
  Serial.println(F("\nDone"));
}

void loop() {
  motor.loopFOC();
  motor.move(0);
  delay(100);
}