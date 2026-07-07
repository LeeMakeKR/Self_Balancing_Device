// MKS ESP32 FOC V2.0 개루프 속도 제어 예제 테스트 라이브러리: SimpleFOC 2.2.1 테스트 하드웨어: MKS ESP32 FOC V2.0

// !!!주의 사항!!!
// ① 직렬 포트에서 "T+숫자"를 입력하여 두 모터의 회전 속도를 설정합니다. 예를 들어 모터를 10rad/s로 설정하려면 "T10"을 입력하고, 모터 전원 인가 시 기본값은 5rad/s로 회전합니다.
// ② 자신의 모터를 사용할 때는 반드시 기본 극 쌍 수를 수정해야 합니다. 즉, BLDCMotor(7)의 값을 자신의 모터의 극 쌍 수로 설정하세요.
// ③ 선택한 모터에 따라 voltage_limit 값을 올바르게 설정하세요. 항공 모형 모터는 0.5~1.0 사이에 설정하고, 짐벌 모터는 4 이하로 설정하는 것이 좋습니다. 너무 큰 전압과 전류는 드라이버 보드를 태울 수 있습니다!
// ④ 개루프 제어는 불가피하게 발열 현상을 야기하므로 이 예제를 1분 이상 실행하지 마십시오. 과열은 모터 또는 드라이버 보드를 태울 수 있습니다!

#include <SimpleFOC.h>


BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25);

// 목표 변수

float target_velocity = 5;
uint32_t prev_millis;

// 저전압 경보 설정
#define UNDERVOLTAGE_THRES 11.1

// 직렬 포트 명령 설정
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}

void board_check();
float get_vin_Volt();
void board_init();
bool flag_under_voltage = false;


void setup() {
  Serial.begin(115200);
  board_init();

  driver.voltage_power_supply = get_vin_Volt();
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_limit = 0.5;    // [V] 이 값을 신중하게 수정하고 확인하세요. 과도한 전압과 전류는 드라이버 보드를 태울 수 있습니다!!!
  motor.velocity_limit = 30;  // [rad/s]


  
  // 개루프 제어 모드 설정
  motor.controller = MotionControlType::velocity_openloop;

  // 하드웨어 초기화
  motor.init();

  // T 명령 추가
  command.add('T', doTarget, "target velocity");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
  motor.move(target_velocity);

  // 전압이 설정값 아래일 때 모터 비활성화
  board_check();

  // 사용자 통신
  if (!flag_under_voltage)
    command.run();
}

void board_init() {
  pinMode(32, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);
  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);

  analogReadResolution(12);  //12bit

  float VIN_Volt = get_vin_Volt();
  while (VIN_Volt <= UNDERVOLTAGE_THRES) {
    VIN_Volt = get_vin_Volt();
    delay(500);
    Serial.printf("전원 인가 대기 중, 현재 전압 %.2f\n", VIN_Volt);
  }
  Serial.printf("모터 캘리브레이션 중... 현재 전압 %.2f\n", VIN_Volt);
}

float get_vin_Volt() {
  return analogReadMilliVolts(13) * 8.5 / 1000;
}

void board_check() {

  uint32_t curr_millis = millis();
  static uint8_t enableState = 0;

  if (curr_millis - prev_millis >= 1000) {
    float vin_Volt = get_vin_Volt();

    if (vin_Volt < UNDERVOLTAGE_THRES) {
      flag_under_voltage = true;
      enableState = 0;
      uint8_t count = 5;
      while (count--) {
        vin_Volt = get_vin_Volt();
        if (vin_Volt > UNDERVOLTAGE_THRES) {
          flag_under_voltage = false;
          break;
        }
      }
    } else {
      flag_under_voltage = false;
    }
    if (flag_under_voltage) {
      motor.disable();
    } else if (0 == enableState && flag_under_voltage == false) {
      enableState = 1;
      motor.enable();
    }
    prev_millis = curr_millis;
  }
}
