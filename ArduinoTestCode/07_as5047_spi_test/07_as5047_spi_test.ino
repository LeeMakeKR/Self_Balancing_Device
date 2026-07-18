// AS5047P (SPI 자기각 센서) 동작 확인 테스트
// MKS ESP32 FOC Mega 보드의 U6 SPI 확장 커넥터 핀 사용
// (ESP32 기본 VSPI 핀과 동일: SCLK=18, MISO=19, MOSI=23, CS=5)

#include <SimpleFOC.h>

#define PIN_SCLK 18
#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_CS   5

// AS5047용 SimpleFOC 기본 설정 사용 (14비트 분해능, 각도 레지스터 0x3FFF)
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, PIN_CS);

void setup() {
  Serial.begin(115200);

  // ESP32 VSPI 핀을 명시적으로 지정하여 SPI 시작
  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  sensor.init();

  Serial.println(F("AS5047P SPI 센서 테스트 시작."));
  Serial.println(F("모터/센서 축을 손으로 천천히 돌려보면서 각도 값이 매끄럽게 변하는지 확인하세요."));
  Serial.println(F("값이 0 또는 특정 값에서 전혀 안 움직이면 CS/SCLK/MISO/MOSI/3.3V/GND 배선을 다시 확인하세요."));
  delay(500);
}

void loop() {
  sensor.update();

  Serial.print("Angle(rad): ");
  Serial.print(sensor.getAngle(), 4);
  Serial.print("\tAngle(deg): ");
  Serial.print(sensor.getAngle() * 180.0f / PI, 2);
  Serial.print("\tVelocity(rad/s): ");
  Serial.println(sensor.getVelocity(), 4);

  delay(100);
}
