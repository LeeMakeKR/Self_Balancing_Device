// MKS ESP32 FOC 인라인 전류 감지 테스트 예제 | 테스트 하드웨어: MKS ESP32 FOC V1.0
// 이 예제에서 측정되는 데이터는 모터 세 상선의 실시간 전류입니다.
// 시리얼 모니터에서 샘플링 데이터를 확인할 수 있습니다.
// 시리얼 플로터에서 실시간 데이터 그래프를 확인할 수 있습니다.

#include <SimpleFOC.h>

// 전류 감지
// 샘플링 저항값  게인  ADC 핀
InlineCurrentSense current_sense0 = InlineCurrentSense(0.01, 50.0, 39, 36);
InlineCurrentSense current_sense1 = InlineCurrentSense(0.01, 50.0, 35, 34);


void setup() {
  // 전류 감지
  current_sense0.init();
  current_sense1.init();

  current_sense0.gain_b *= -1;
  current_sense1.gain_b *= -1;
  
  Serial.begin(115200);
  Serial.println("Current sense ready.");
}

void loop() {

  PhaseCurrent_s currents0 = current_sense0.getPhaseCurrents();
  float current_magnitude0 = current_sense0.getDCCurrent();
  PhaseCurrent_s currents1 = current_sense1.getPhaseCurrents();
  float current_magnitude1 = current_sense1.getDCCurrent();

  Serial.print(currents0.a*1000); // milli Amps
  Serial.print("\t");
  Serial.print(currents0.b*1000); // milli Amps
  Serial.print("\t");
  Serial.print(currents0.c*1000); // milli Amps
  Serial.print("\t");
  Serial.println(current_magnitude0*1000); // milli Amps
  Serial.print(currents1.a*1000); // milli Amps
  Serial.print("\t");
  Serial.print(currents1.b*1000); // milli Amps
  Serial.print("\t");
  Serial.print(currents1.c*1000); // milli Amps
  Serial.print("\t");
  Serial.println(current_magnitude1*1000); // milli Amps
  Serial.println();
}
