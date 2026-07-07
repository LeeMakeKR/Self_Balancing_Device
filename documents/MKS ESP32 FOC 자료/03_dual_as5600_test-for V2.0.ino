// MKS ESP32 FOC AS5600 테스트 예제 | 테스트 하드웨어: MKS ESP32 FOC V1.0
// 모터를 수동으로 회전시키면 시리얼 모니터에서 모터 위치를 확인할 수 있습니다.
// 첫 번째 열과 두 번째 열은 각각 M0과 M1 모터의 위치 좌표입니다.

#include <SimpleFOC.h>


MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);


void setup() {

  
  Serial.begin(115200);
  _delay(750);
  I2Cone.begin(19,18, 400000);   //SDA0,SCL0
  I2Ctwo.begin(23,5, 400000);
  
  //최신 버전 ESP-Arduino 2.0.2의 경우, 아래 두 줄을 사용하세요.
  //I2Cone.begin(19,18, 400000UL);   //SDA0,SCL0
  //I2Ctwo.begin(23,5, 400000UL);
  
  sensor0.init(&I2Cone);
  sensor1.init(&I2Ctwo);
}

void loop() {
  // sensor0.update(); // simplefoc 라이브러리 버전이 2.20 이상이면 이 두 줄의 주석을 해제하세요.
  // sensor1.update();
  //_delay(200);
  Serial.print(sensor0.getAngle()); 
  Serial.print(" - "); 
  Serial.print(sensor1.getAngle());
  Serial.println();
}
