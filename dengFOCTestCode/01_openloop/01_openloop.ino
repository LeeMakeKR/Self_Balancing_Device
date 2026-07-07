#include <Arduino.h>
#include <Wire.h>
#include "AS5600.h"

// 값을 [low, high] 범위로 제한하는 매크로
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// 3π/2 상수 (센서 정렬 시 사용)
#define _3PI_2 4.71238898038f

// I2C 핀 정의 (ESP32-C3)
#define SDA_PIN 6
#define SCL_PIN 5

float voltage_power_supply;
float Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0;
float zero_electric_angle = 0;
int PP = 1, DIR = 1;

// PWM 출력 핀 정의 (ESP32-C3)
int pwmA = 3;
int pwmB = 1;
int pwmC = 7;
int enablePin = 4;


// 각도를 [0, 2π] 범위로 정규화
// fmod 결과가 음수일 수 있으므로 음수 시 2π를 더해 보정
float _normalizeAngle(float angle) {
  float a = fmod(angle, 2 * PI);
  return a >= 0 ? a : (a + 2 * PI);
}


// 듀티비 계산 후 PWM 출력
// ESP32-C3 코어 v3.x: 채널 번호 대신 핀 직접 지정
void setPwm(float Ua, float Ub, float Uc) {
  // 각 상의 듀티비를 0.0~1.0 범위로 제한
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  // 핀에 직접 PWM 값 쓰기 (8비트: 0~255)
  ledcWrite(pwmA, dc_a * 255);
  ledcWrite(pwmB, dc_b * 255);
  ledcWrite(pwmC, dc_c * 255);
}

// 역파크 변환 + 역클라크 변환으로 3상 전압 계산 후 PWM 출력
void setTorque(float Uq, float angle_el) {
  Uq = _constrain(Uq, -voltage_power_supply / 2, voltage_power_supply / 2);
  angle_el = _normalizeAngle(angle_el);

  // 역파크 변환 (회전좌표계 dq → 정지좌표계 αβ)
  Ualpha = -Uq * sin(angle_el);
  Ubeta  =  Uq * cos(angle_el);

  // 역클라크 변환 (αβ → 3상 abc), 중간 전압 오프셋 적용
  Ua = Ualpha + voltage_power_supply / 2;
  Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
  Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;
  setPwm(Ua, Ub, Uc);
}

// FOC 드라이버 초기화 (공급 전압 설정, PWM 및 I2C 초기화)
void DFOC_Vbus(float power_supply) {
  voltage_power_supply = power_supply;

  // 모터 드라이버 활성화 핀 설정
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);

  // I2C 초기화 (AS5600 센서용)
  Wire.begin(SDA_PIN, SCL_PIN);

  // PWM 초기화: ledcAttach(핀, 주파수, 해상도)
  // ESP32-C3 코어 v3.x 방식 — 채널 번호 불필요
  ledcAttach(pwmA, 30000, 8);  // 30kHz, 8비트 해상도
  ledcAttach(pwmB, 30000, 8);
  ledcAttach(pwmC, 30000, 8);

  Serial.println("PWM 초기화 완료");
  BeginSensor();
}


// 현재 전기각 계산 (센서값 기반)
float _electricalAngle() {
  return _normalizeAngle((float)(DIR * PP) * getAngle_Without_track() - zero_electric_angle);
}


// 센서와 극쌍수를 이용해 영점 전기각 보정
void DFOC_alignSensor(int _PP, int _DIR) {
  PP = _PP;
  DIR = _DIR;
  setTorque(3, _3PI_2);   // 알려진 각도로 고정 자장 인가
  delay(3000);
  zero_electric_angle = _electricalAngle();  // 현재 위치를 영점으로 기록
  setTorque(0, _3PI_2);
  Serial.print("영점 전기각: "); Serial.println(zero_electric_angle);
}

// 모터 현재 기계각 반환
float DFOC_M0_Angle() {
  return getAngle();
}


//==============시리얼 수신==============
float motor_target;
int commaPosition;

// 시리얼로 수신된 명령 파싱 후 motor_target 갱신
String serialReceiveUserCommand() {
  static String received_chars;
  String command = "";

  while (Serial.available()) {
    char inChar = (char)Serial.read();
    received_chars += inChar;

    if (inChar == '\n') {
      command = received_chars;

      commaPosition = command.indexOf('\n');  // 줄바꿈 문자 위치 탐색
      if (commaPosition != -1) {
        motor_target = command.substring(0, commaPosition).toDouble();  // 목표값 파싱
        Serial.println(motor_target);
      }
      received_chars = "";
    }
  }
  return command;
}

// 현재 motor_target 값 반환
float serial_motor_target() {
  return motor_target;
}
