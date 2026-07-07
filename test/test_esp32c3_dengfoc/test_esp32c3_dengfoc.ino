#include <SimpleFOC.h>

// Example of AS5600 configuration 
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

void setup() {
  // monitoring port
  Serial.begin(115200);

  // ESP32-C3 사용자 지정 I2C 핀 초기화 (SDA: 6, SCL: 5)
  Wire.begin(6, 5);
  // configure i2C
  Wire.setClock(400000);
  
  // initialise magnetic sensor hardware
  sensor.init();

  Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and 
  // has to be called before getAngle nad getVelocity
  sensor.update();
  
  // display the angle and the angular velocity to the terminal
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
}