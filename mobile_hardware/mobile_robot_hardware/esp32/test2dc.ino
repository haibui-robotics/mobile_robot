#include <Arduino.h>
// [THÊM 2026-11-02] Thư viện giao tiếp I2C chuẩn của Arduino/ESP32 để đọc cảm biến MPU6050
// (Không cần cài thêm thư viện ngoài trên Arduino IDE, tránh lỗi thiếu thư viện)
#include <Wire.h>

// =========================================================================
// FIRMWARE ĐIỀU KHIỂN 2 ĐỘNG CƠ ENCODER + IMU MPU6050 KẾT NỐI ROS 2 (ESP32)
// Chuẩn điều khiển mượt mà: Feedforward + Bounded-I PI + Tự động hiệu chuẩn IMU
// =========================================================================

// ===== [THÊM 2026-11-02] CẤU HÌNH CẢM BIẾN IMU MPU6050 (GIAO TIẾP I2C) =====
const int I2C_SDA = 21;          // Chân SDA mặc định của ESP32
const int I2C_SCL = 22;          // Chân SCL mặc định của ESP32
const uint8_t MPU_ADDR = 0x68;   // Địa chỉ I2C mặc định của MPU6050 (chân AD0 nối GND hoặc để trống)

bool imu_connected = false;      // Cờ kiểm tra MPU6050 có kết nối thành công hay không

// Các biến lưu sai số tĩnh (Offset) của MPU6050 sau khi hiệu chuẩn (Calibration)
float gyro_offset_x = 0.0f;
float gyro_offset_y = 0.0f;
float gyro_offset_z = 0.0f;
float accel_offset_x = 0.0f;
float accel_offset_y = 0.0f;
float accel_offset_z = 0.0f;

// Dữ liệu IMU đã hiệu chuẩn và lọc nhiễu theo chuẩn đơn vị ROS 2:
// - Gia tốc dài (Linear Acceleration): m/s^2
// - Vận tốc góc (Angular Velocity): rad/s
// - Góc hướng Yaw tích phân từ Gyro Z: rad
float imu_ax = 0.0f, imu_ay = 0.0f, imu_az = 9.80665f;
float imu_gx = 0.0f, imu_gy = 0.0f, imu_gz = 0.0f;
float imu_yaw = 0.0f;
const float GRAVITY_MSS = 9.80665f;       // Gia tốc trọng trường chuẩn (1g = 9.80665 m/s^2)
const float DEG_TO_RAD_FACTOR = PI / 180.0f;

// ===== CẤU HÌNH CHÂN DRIVER TB6612 =====
const int PWMA = 32;     // PWM Kênh A (Động cơ TRÁI)
const int AIN1 = 33;     // Hướng Kênh A
const int AIN2 = 25;     // Hướng Kênh A

const int PWMB = 26;     // PWM Kênh B (Động cơ PHẢI)
const int BIN1 = 27;     // Hướng Kênh B
const int BIN2 = 14;     // Hướng Kênh B

const int STBY = 12;     // Chân kích hoạt TB6612 (HIGH để mở cầu H)

// ===== CẤU HÌNH CHÂN ENCODER =====
const int ENC_L_A = 18;  // Kênh A Encoder TRÁI
const int ENC_L_B = 19;  // Kênh B Encoder TRÁI
const int ENC_R_A = 5;   // Kênh A Encoder PHẢI
const int ENC_R_B = 4;   // Kênh B Encoder PHẢI

// ===== THÔNG SỐ VẬT LÝ =====
const float TOTAL_CPR = 1234.3; // Tổng số xung / 1 vòng quay bánh xe

// Biến đếm xung encoder
volatile long countLeft = 0;
volatile long countRight = 0;

// Mốc xung chu kỳ trước
long prevCountLeft = 0;
long prevCountRight = 0;

// Vận tốc góc mục tiêu từ ROS 2 (rad/s)
float target_vel_L = 0.0;
float target_vel_R = 0.0;

// Vận tốc góc đo được sau khi lọc (rad/s)
float measured_vel_L = 0.0;
float measured_vel_R = 0.0;

// ===== HỆ SỐ ĐIỀU KHIỂN PI & FEEDFORWARD =====
// [SỬA]: Tăng Kp, Ki, Kff và min_pwm để có đủ momen xoắn (torque) thắng tải trọng xe khi đặt xuống đất.
// Giá trị cũ khi chạy không tải trên không: Kff = 25.0, Kp = 8.0, Ki = 6.0, min_pwm = 28.0
float Kff = 30.0;        // [SỬA từ 25.0 -> 30.0]: Tăng lực đẩy cơ bản theo vận tốc đặt
float Kp  = 40.0;        // [SỬA từ 8.0  -> 40.0]: Phản ứng mạnh ngay khi bánh xe bị ma sát mặt đất ghì chậm lại
float Ki  = 80.0;        // [SỬA từ 6.0  -> 80.0]: Nhanh chóng bơm thêm PWM bù tải nặng dưới mặt đất
float min_pwm = 45.0;    // [SỬA từ 28.0 -> 45.0]: Mức PWM tối thiểu đủ thắng ma sát lăn khi xe chạm đất

float error_integral_L = 0.0;
float error_integral_R = 0.0;

// Chu kỳ vòng lặp PID (50 Hz = 20 ms)
unsigned long last_control_time = 0;
const unsigned long CONTROL_PERIOD_MS = 20;

// Safety timeout: Mất tín hiệu ROS 2 quá 1 giây sẽ tự dừng
unsigned long last_cmd_time = 0;

String rx_buffer = "";

// ===== HÀM NGẮT ĐỌC ENCODER =====
void IRAM_ATTR isrEncoderLeft() {
  if (digitalRead(ENC_L_B) == HIGH) {
    countLeft--;  // Đảo dấu để khi quay tiến, xung đếm TĂNG (+)
  } else {
    countLeft++;
  }
}

void IRAM_ATTR isrEncoderRight() {
  if (digitalRead(ENC_R_B) == HIGH) {
    countRight++;
  } else {
    countRight--;
  }
}

// ===== XUẤT PWM ĐỘNG CƠ TRÁI =====
void setMotorLeft(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, pwm);
  } else if (pwm < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -pwm);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
}

// ===== XUẤT PWM ĐỘNG CƠ PHẢI =====
void setMotorRight(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, pwm);
  } else if (pwm < 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, -pwm);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}

// ===== BỘ TÍNH TOÁN PI VẬN TỐC (CHỐNG NGHẼN TÍCH PHÂN & CHỐNG GIẬT) =====
int computePI(float target_vel, float measured_vel, float &integral, float dt) {
  // Khi lệnh dừng, ngắt hoàn toàn và reset sạch bộ tích phân
  if (abs(target_vel) < 0.01) {
    integral = 0.0;
    return 0;
  }

  float error = target_vel - measured_vel;
  integral += error * dt;

  // [SỬA - KẸP TÍCH PHÂN BẤT ĐỐI XỨNG (Asymmetric Anti-Windup)]:
  // Code cũ: i_term = constrain(i_term, -20.0f, 20.0f); -> Bị giới hạn trần ở +20 PWM nên khi đặt
  // xe xuống đất (tải nặng cần 120-200 PWM) thì bộ điều khiển không thể bơm thêm điện áp!
  // Giải pháp mới:
  //   - Khi chạy TIẾN (target_vel > 0): Cho phép i_term tăng tới +200 PWM để thắng sức nặng của xe,
  //     nhưng chặn đáy âm ở -20 PWM để không bao giờ bị hố âm làm lịm xe.
  //   - Khi chạy LÙI (target_vel < 0): Cho phép i_term xuống tới -200 PWM, chặn trần dương ở +20 PWM.
  float i_term = Ki * integral;
  if (target_vel > 0.0) {
    i_term = constrain(i_term, -20.0f, 200.0f);
  } else {
    i_term = constrain(i_term, -200.0f, 20.0f);
  }
  integral = i_term / Ki; // Đồng bộ lại biến tích phân chống tràn (Anti-windup)

  // Output = Feedforward (chạy đúng tốc độ ngay) + P (bù sai số) + I (bù tải nhẹ)
  float output = (target_vel * Kff) + (Kp * error) + i_term;

  // Chống đảo chiều đột ngột khi đang chạy
  if (target_vel > 0.0) {
    if (output < 0.0) output = 0.0;
    if (output > 0.0 && output < min_pwm) output = min_pwm;
    return constrain((int)output, 0, 255);
  } else {
    if (output > 0.0) output = 0.0;
    if (output < 0.0 && output > -min_pwm) output = -min_pwm;
    return constrain((int)output, -255, 0);
  }
}

// ===== [THÊM 2026-11-02] HÀM KHỞI TẠO VÀ HIỆU CHUẨN CẢM BIẾN MPU6050 =====
bool initMPU6050() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // Chạy I2C tốc độ cao 400kHz để đọc cảm biến cực nhanh (< 0.4ms), không làm trễ PID

  // 1. Đánh thức MPU6050 (Ghi 0x00 vào thanh ghi PWR_MGMT_1 - 0x6B)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission(true) != 0) {
    return false; // Không tìm thấy MPU6050 trên đường truyền I2C
  }

  // 2. Bật bộ lọc thông thấp phần cứng DLPF (Thanh ghi CONFIG 0x1A = 0x03 -> Bandwidth ~44Hz)
  // Giúp triệt tiêu nhiễu rung cơ khí từ 2 động cơ DC truyền lên khung xe
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);

  // 3. Cấu hình dải đo Gyroscope: ±250 deg/s (Thanh ghi GYRO_CONFIG 0x1B = 0x00 -> Độ nhạy 131.0 LSB/deg/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // 4. Cấu hình dải đo Accelerometer: ±2g (Thanh ghi ACCEL_CONFIG 0x1C = 0x00 -> Độ nhạy 16384.0 LSB/g)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(50);
  return true;
}

// Hàm tự động lấy mẫu khi xe đứng yên để tìm độ lệch tĩnh (Offset Calibration)
void calibrateMPU6050(int num_samples = 200) {
  if (!imu_connected) return;

  // Dừng động cơ tuyệt đối trong lúc hiệu chuẩn
  setMotorLeft(0);
  setMotorRight(0);

  double sum_ax = 0, sum_ay = 0, sum_az = 0;
  double sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int valid_samples = 0;

  for (int i = 0; i < num_samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Bắt đầu đọc từ thanh ghi ACCEL_XOUT_H (0x3B)
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true) == 14) {
      int16_t raw_ax = (Wire.read() << 8) | Wire.read();
      int16_t raw_ay = (Wire.read() << 8) | Wire.read();
      int16_t raw_az = (Wire.read() << 8) | Wire.read();
      Wire.read(); Wire.read(); // Bỏ qua 2 byte nhiệt độ
      int16_t raw_gx = (Wire.read() << 8) | Wire.read();
      int16_t raw_gy = (Wire.read() << 8) | Wire.read();
      int16_t raw_gz = (Wire.read() << 8) | Wire.read();

      sum_ax += raw_ax;
      sum_ay += raw_ay;
      sum_az += raw_az;
      sum_gx += raw_gx;
      sum_gy += raw_gy;
      sum_gz += raw_gz;
      valid_samples++;
    }
    delay(3); // Lấy mẫu cách nhau 3ms (~0.6 giây cho 200 mẫu)
  }

  if (valid_samples > 0) {
    // Khi xe đứng yên trên mặt phẳng ngang:
    // - Gyro chuẩn phải bằng 0 (rad/s) ở cả 3 trục X, Y, Z
    // - Accel chuẩn phải bằng 0 ở trục X, Y và bằng +16384 LSB (+1g) ở trục Z
    accel_offset_x = (float)(sum_ax / valid_samples);
    accel_offset_y = (float)(sum_ay / valid_samples);
    accel_offset_z = (float)(sum_az / valid_samples) - 16384.0f;

    gyro_offset_x = (float)(sum_gx / valid_samples);
    gyro_offset_y = (float)(sum_gy / valid_samples);
    gyro_offset_z = (float)(sum_gz / valid_samples);

    imu_yaw = 0.0f; // Reset góc hướng Yaw về 0 sau khi hiệu chuẩn
  }
}

// Hàm đọc và quy đổi dữ liệu MPU6050 sang đơn vị chuẩn của ROS 2 (m/s^2 và rad/s)
void readMPU6050(float dt) {
  if (!imu_connected) return;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;

  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true) == 14) {
    int16_t raw_ax = (Wire.read() << 8) | Wire.read();
    int16_t raw_ay = (Wire.read() << 8) | Wire.read();
    int16_t raw_az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Bỏ qua thanh ghi nhiệt độ
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();

    // 1. Trừ Offset hiệu chuẩn và đổi sang đơn vị chuẩn SI:
    // Accel: chia 16384.0 (LSB/g) rồi nhân 9.80665 (m/s^2)
    float ax_new = ((float)raw_ax - accel_offset_x) / 16384.0f * GRAVITY_MSS;
    float ay_new = ((float)raw_ay - accel_offset_y) / 16384.0f * GRAVITY_MSS;
    float az_new = ((float)raw_az - accel_offset_z) / 16384.0f * GRAVITY_MSS;

    // Gyro: chia 131.0 (LSB/deg/s) rồi nhân (PI / 180) để ra rad/s
    float gx_new = ((float)raw_gx - gyro_offset_x) / 131.0f * DEG_TO_RAD_FACTOR;
    float gy_new = ((float)raw_gy - gyro_offset_y) / 131.0f * DEG_TO_RAD_FACTOR;
    float gz_new = ((float)raw_gz - gyro_offset_z) / 131.0f * DEG_TO_RAD_FACTOR;

    // 2. Bộ lọc thông thấp (Low-pass filter) làm mượt gia tốc và vận tốc góc
    imu_ax = 0.7f * imu_ax + 0.3f * ax_new;
    imu_ay = 0.7f * imu_ay + 0.3f * ay_new;
    imu_az = 0.7f * imu_az + 0.3f * az_new;

    imu_gx = 0.7f * imu_gx + 0.3f * gx_new;
    imu_gy = 0.7f * imu_gy + 0.3f * gy_new;

    // 3. Vùng chết (Deadband) cho trục Z (Yaw rate):
    // Nếu vận tốc góc nhỏ hơn 0.015 rad/s (~0.85 độ/s) khi xe đang đứng yên thì ép về 0.0
    // để góc Yaw không bị trôi (drift) theo thời gian trước khi đưa vào Kalman Filter.
    if (abs(gz_new) < 0.015f) {
      imu_gz = 0.0f;
    } else {
      imu_gz = 0.7f * imu_gz + 0.3f * gz_new;
    }

    // 4. Tích phân vận tốc góc Z để ước lượng góc quay Yaw (rad)
    imu_yaw += imu_gz * dt;
  }
}

// ===== XỬ LÝ DỮ LIỆU SERIAL TỪ ROS 2 =====
void processSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rx_buffer.length() > 0) {
        // [THÊM 2026-11-02] Hỗ trợ lệnh "CAL" qua Serial để hiệu chuẩn lại MPU6050 bất cứ lúc nào
        if (rx_buffer == "CAL") {
          calibrateMPU6050(200);
          rx_buffer = "";
          continue;
        }

        float l_in = 0.0, r_in = 0.0;
        // ROS 2 gửi định dạng: "L%.2f R%.2f\n"
        if (sscanf(rx_buffer.c_str(), "L%f R%f", &l_in, &r_in) == 2) {
          target_vel_L = l_in;
          target_vel_R = r_in;
          last_cmd_time = millis();

          // Khi nhận lệnh dừng, dọn sạch trạng thái tích phân
          if (abs(l_in) < 0.01) {
            error_integral_L = 0.0;
            measured_vel_L = 0.0;
          }
          if (abs(r_in) < 0.01) {
            error_integral_R = 0.0;
            measured_vel_R = 0.0;
          }
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

  // Cấu hình chân Motor
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // Bật TB6612

  // Cấu hình chân Encoder
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncoderRight, RISING);

  // [THÊM 2026-11-02] Khởi tạo và tự động hiệu chuẩn MPU6050 ngay khi cấp nguồn/khởi động
  // Lưu ý: Giữ xe đứng yên trong ~1 giây đầu khi mới bật nguồn hoặc khi khởi chạy ROS 2
  imu_connected = initMPU6050();
  if (imu_connected) {
    calibrateMPU6050(200);
  }

  last_cmd_time = millis();
  last_control_time = millis();
}

void loop() {
  // 1. Nhận lệnh vận tốc góc từ ROS 2
  processSerial();

  // 2. Safety Timeout: Tự dừng sau 1 giây nếu mất tín hiệu từ ROS 2
  if (millis() - last_cmd_time > 1000) {
    target_vel_L = 0.0;
    target_vel_R = 0.0;
    error_integral_L = 0.0;
    error_integral_R = 0.0;
  }

  // 3. Vòng lặp điều khiển chu kỳ 20ms (50 Hz)
  unsigned long now = millis();
  if (now - last_control_time >= CONTROL_PERIOD_MS) {
    float dt = (now - last_control_time) / 1000.0;
    last_control_time = now;
    if (dt <= 0.0) dt = 0.02;

    // Đọc số xung an toàn (Atomic read)
    noInterrupts();
    long cur_L = countLeft;
    long cur_R = countRight;
    interrupts();

    long d_L = cur_L - prevCountLeft;
    long d_R = cur_R - prevCountRight;
    prevCountLeft = cur_L;
    prevCountRight = cur_R;

    // Vận tốc góc thô đo được (rad/s)
    float raw_vel_L = (float(d_L) / TOTAL_CPR) * (2.0 * PI) / dt;
    float raw_vel_R = (float(d_R) / TOTAL_CPR) * (2.0 * PI) / dt;

    // Lọc mượt vận tốc đo được
    measured_vel_L = 0.65f * measured_vel_L + 0.35f * raw_vel_L;
    measured_vel_R = 0.65f * measured_vel_R + 0.35f * raw_vel_R;

    // Tính toán PWM mượt mà
    int pwmL = computePI(target_vel_L, measured_vel_L, error_integral_L, dt);
    int pwmR = computePI(target_vel_R, measured_vel_R, error_integral_R, dt);

    setMotorLeft(pwmL);
    setMotorRight(pwmR);

    // [THÊM 2026-11-02] Đọc dữ liệu cảm biến IMU MPU6050 đã hiệu chuẩn
    readMPU6050(dt);

    // 4. Phản hồi số xung encoder + dữ liệu IMU về ROS 2 trên cùng 1 dòng Serial (50 Hz)
    // Định dạng: "L<xung_trái> R<xung_phải> AX<ax> AY<ay> AZ<az> GX<gx> GY<gy> GZ<gz> YAW<yaw>\n"
    Serial.print("L");
    Serial.print(cur_L);
    Serial.print(" R");
    Serial.print(cur_R);
    if (imu_connected) {
      Serial.print(" AX"); Serial.print(imu_ax, 3);
      Serial.print(" AY"); Serial.print(imu_ay, 3);
      Serial.print(" AZ"); Serial.print(imu_az, 3);
      Serial.print(" GX"); Serial.print(imu_gx, 4);
      Serial.print(" GY"); Serial.print(imu_gy, 4);
      Serial.print(" GZ"); Serial.print(imu_gz, 4);
      Serial.print(" YAW"); Serial.println(imu_yaw, 4);
    } else {
      Serial.println();
    }
  }
}
