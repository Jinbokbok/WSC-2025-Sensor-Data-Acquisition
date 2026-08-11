#include <LiquidCrystal_I2C.h>
#include <mcp_can.h>
#include <SPI.h>

// =======================
// LCD & CAN 설정
// =======================
LiquidCrystal_I2C lcd(0x27, 16, 2);
MCP_CAN CAN0(53);  // CS: 핀 53

#define PWM_PIN 9  // PWM 출력 핀

// =======================
// 물리 파라미터
// =======================
const int vpulse = 48;  // 바퀴 1회전 당 펄스 수
double wheelDiameter = 0.55;
double wheelPerimeter = 3.14159 * wheelDiameter;  // 바퀴 둘레 (m)

// =======================
// 아날로그 핀, 임계값
// =======================
const int analogPin = A0;
const float thresholdVoltage = 2.0;  // 2.0V 이상이면 'HIGH'로 간주

// =======================
// 소프트웨어 펄스 간격 측정용
// =======================
unsigned long prevPulseMicros = 0;  // 직전 펄스 시각 (µs)
unsigned long dtMicros = 0;         // 마지막 펄스 간격 (µs)

bool lastState = false;             // 이전 "디지털화된" 신호 상태
bool newDtAvailable = false;        // 새 dt가 준비됐는지 표시
unsigned long lastPulseMillis = 0;  // 타임아웃(무펄스) 확인 용도

// 속도/회전수 결과
double currentSpeed = 0.0;  // km/h
double currentRPM = 0.0;

// 무펄스 타임아웃 (1초)
const unsigned long timeoutMs = 1000;

// LCD & CAN 업데이트 주기
unsigned long prevDisplayMs = 0;
const unsigned long displayInterval = 200;  // 0.2초

//PID 관련

float targetSpeed = 0;  // 목표 속도
float error, lastError = 0.0;
float integral = 0.0, derivative = 0.0;
float output = 0.0;

// PID 게인 값
float Kp = 2.0;
float Ki = 0.1;
float Kd = 0.01;

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  // CAN 초기화
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515...");
    while (1)
      ;
  }
  CAN0.setMode(MCP_NORMAL);
  pinMode(PWM_PIN, OUTPUT);
}

void loop() {
  // ----------------------------------------------------------------
  // 1) 아날로그 읽어 임계값 이상이면 HIGH 판단
  // ----------------------------------------------------------------
  int sensorValue = analogRead(analogPin);            // 0~1023
  float voltage = sensorValue * (5.0 / 1023.0);       // 0~5.0V 변환
  bool currentState = (voltage >= thresholdVoltage);  // Threshold 비교

  // Serial.print("volate: ");
  // Serial.println(float(voltage));
  // ----------------------------------------------------------------
  // 2) 상승엣지(LOW->HIGH) 감지해 펄스 간격(dt) 계산
  // ----------------------------------------------------------------
  if (!lastState && currentState) {
    // → 지금 순간, '상승엣지' 발생
    unsigned long nowMicros = micros();
    if (prevPulseMicros != 0) {
      // 첫 펄스가 아니라면 dt 계산
      unsigned long tempDt = nowMicros - prevPulseMicros;
      // 소프트웨어 디바운스 (예: 1000us 이하 무시)
      if (tempDt > 1000) {
        dtMicros = tempDt;
        newDtAvailable = true;
        lastPulseMillis = millis();  // 타이밍 기록
      }
    } else {
      // 첫 펄스인 경우, 시간 기록만
      lastPulseMillis = millis();
    }
    // 갱신
    prevPulseMicros = nowMicros;
  }
  lastState = currentState;

  // ----------------------------------------------------------------
  // 3) 메인루프에서 dt를 이용해 속도계산 (필요 시)
  // ----------------------------------------------------------------
  if (newDtAvailable) {
    // 원자적 접근이 필요할 정도는 아니지만, 혹시나...
    noInterrupts();
    unsigned long localDt = dtMicros;
    newDtAvailable = false;
    interrupts();

    double rotationTime_us = (double)localDt * vpulse;  // 바퀴 1회전 시간 (µs)
    double rotationTime_s = rotationTime_us / 1e6;      // (초)
    if (rotationTime_s > 0) {
      double speed_m_s = wheelPerimeter / rotationTime_s;  // m/s
      currentSpeed = speed_m_s * 3.6;                      // km/h
      currentRPM = (1.0 / rotationTime_s) * 60.0;          // RPM
    }
  }

  // ----------------------------------------------------------------
  // 4) 타임아웃 (무펄스) 처리
  // ----------------------------------------------------------------
  if ((millis() - lastPulseMillis) > timeoutMs && prevPulseMicros != 0) {
    // 1초 이상 펄스가 없으면 -> 속도 0
    currentSpeed = 0.0;
    currentRPM = 0.0;
  }

  // ----------------------------------------------------------------
  // todo: 주기적으로 LCD, CAN 출력
  // ----------------------------------------------------------------
  if ((millis() - prevDisplayMs) >= displayInterval) {
    prevDisplayMs = millis();

    // Serial Debug
    Serial.print("\t");
    Serial.print(currentSpeed);
    Serial.print("\t");
    Serial.println(targetSpeed);
  }

  // 목표 속도 조절 (d: 감소, u: 증가)
  if (Serial.available()) {
    char input = Serial.read();
    if (input == 'd') {
      targetSpeed -= 1.0;
    } else if (input == 'u') {
      targetSpeed += 1.0;
    }
  }

  // PID 계산
  error = targetSpeed - currentSpeed;
  integral += error;
  derivative = error - lastError;
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  output = constrain(output, 0, 255);  // PWM 값 제한

  lastError = error;

  // PWM 출력
  analogWrite(PWM_PIN, output);
}
