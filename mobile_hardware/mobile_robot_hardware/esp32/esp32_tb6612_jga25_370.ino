#include <Arduino.h>

// ===== CHÂN ĐIỀU KHIỂN MOTOR (TB6612) =====
const int PWMA = 32;     // Kênh A - Động cơ TRÁI
const int AIN1 = 25;
const int AIN2 = 33;

const int PWMB = 26;     // Kênh B - Động cơ PHẢI
const int BIN1 = 27;
const int BIN2 = 14;

// ===== CẤU HÌNH PWM LEDC DÀNH RIÊNG CHO ESP32 =====
const int PWM_FREQ = 10000;    // Tần số PWM 10kHz (Tránh tiếng hú động cơ)
const int PWM_RES = 8;         // Độ phân giải 8-bit (0 - 255)
const int CHANNEL_L = 0;       // Kênh PWM Trái
const int CHANNEL_R = 1;       // Kênh PWM Phải

// ===== CHÂN ENCODER =====
const int ENC_L_A = 18;  // Kênh A Encoder TRÁI
const int ENC_L_B = 19;  // Kênh B Encoder TRÁI

const int ENC_R_A = 22;   // Kênh A Encoder PHẢI
const int ENC_R_B = 23;   // Kênh B Encoder PHẢI

// Tong xung / 1 vong dia xich
const double TOTAL_CPR = 1234.3;

// ===== QUẢN LÝ THỜI GIAN VÒNG LẶP PID =====
unsigned long last_control_time = 0;
const unsigned long CONTROL_PERIOD_MS = 20; // 50 Hz
unsigned long last_cmd_time = 0;

// Biến đếm xung ngắt
volatile long countLeft = 0;
volatile long countRight = 0;

// Mốc xung chu kỳ trước
long prevCountLeft = 0;
long prevCountRight = 0;

// Vận tốc góc mục tiêu (rad/s)
float target_vel_L = 0.0;
float target_vel_R = 0.0;

// Vận tốc góc thực tế (rad/s)
float measured_vel_L = 0.0;
float measured_vel_R = 0.0;

// Biến PID
float error_integral_L = 0.0;
float error_integral_R = 0.0;
float prev_error_L = 0.0;
float prev_error_R = 0.0;

// Hệ số PID
float Kp = 20.0;
float Ki = 10.0;
float Kd = 0.2;

String rx_buffer = "";

// ===== HÀM NGẮT ĐỌC ENCODER =====
void IRAM_ATTR isrEncoderLeft() {
  if (digitalRead(ENC_L_B) == HIGH) {
    countLeft++;
  } else {
    countLeft--;
  }
}

void IRAM_ATTR isrEncoderRight() {
  if (digitalRead(ENC_R_B) == HIGH) {
    countRight++;
  } else {
    countRight--;
  }
}

// ===== HÀM XUẤT PWM SỬ DỤNG LEDC SCHEME =====
void setMotorLeft(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(CHANNEL_L, pwm);
  } else if (pwm < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(CHANNEL_L, -pwm);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    ledcWrite(CHANNEL_L, 0);
  }
}

void setMotorRight(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(BIN1, LOW);  
    digitalWrite(BIN2, HIGH);
    ledcWrite(CHANNEL_R, pwm);
  } else if (pwm < 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(CHANNEL_R, -pwm);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(CHANNEL_R, 0);
  }
}

// ===== TÍNH TOÁN PID VẬN TỐC =====
int computePID(float target_vel, float measured_vel, float &integral, float &prev_error, float dt) {
  if (abs(target_vel) < 0.01) {
    integral = 0.0;
    prev_error = 0.0;
    return 0;
  }

  float error = target_vel - measured_vel;
  integral += error * dt;
  integral = constrain(integral, -15.0, 15.0); // Anti-windup

  float derivative = (error - prev_error) / dt;
  prev_error = error;

  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  return (int)output;
}

void processSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rx_buffer.length() > 0) {
        float l_in = 0.0, r_in = 0.0;
        if (sscanf(rx_buffer.c_str(), "L%f R%f", &l_in, &r_in) == 2) {
          target_vel_L = l_in;
          target_vel_R = r_in;
          last_cmd_time = millis();
        }
        rx_buffer = "";
      }
    } else {
      rx_buffer += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  rx_buffer.reserve(32);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Cấu hình LEDC PWM
  ledcSetup(CHANNEL_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, CHANNEL_L);

  ledcSetup(CHANNEL_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMB, CHANNEL_R);

  // Cấu hình Encoder
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncoderRight, RISING);

  last_cmd_time = millis();
  delay(500); // Chờ 0.5s cho nguồn và phần cứng ổn định
  Serial.println("\n--- ESP32 HARDWARE READY ---");
}

void loop() {
  processSerial();

  // Safety Timeout: Tự dừng sau 1 giây mất kết nối ROS
  if (millis() - last_cmd_time > 1000) {
    target_vel_L = 0.0;
    target_vel_R = 0.0;
  }

  unsigned long now = millis();
  if (now - last_control_time >= CONTROL_PERIOD_MS) {
    float dt = (now - last_control_time) / 1000.0;
    last_control_time = now;
    if (dt <= 0.0) dt = 0.02;

    noInterrupts();
    long currentCountLeft = countLeft;
    long currentCountRight = countRight;
    interrupts();

    long d_left = currentCountLeft - prevCountLeft;
    long d_right = currentCountRight - prevCountRight;

    prevCountLeft = currentCountLeft;
    prevCountRight = currentCountRight;

    measured_vel_L = (d_left / TOTAL_CPR) * (2.0 * M_PI) / dt;
    measured_vel_R = (d_right / TOTAL_CPR) * (2.0 * M_PI) / dt;

    int pwm_L = computePID(target_vel_L, measured_vel_L, error_integral_L, prev_error_L, dt);
    int pwm_R = computePID(target_vel_R, measured_vel_R, error_integral_R, prev_error_R, dt);

    setMotorLeft(pwm_L);
    setMotorRight(pwm_R);

    // Phản hồi xung về ROS 2
    Serial.print("L");
    Serial.print(currentCountLeft);
    Serial.print(" R");
    Serial.println(currentCountRight);
  }
}