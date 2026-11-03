#include <Arduino.h>

const int PWMA = 32;     //LEFT
const int AIN1 = 33;  //25
const int AIN2 = 25;   //33

const int PWMB = 26;     //RIGHT
const int BIN1 = 27;  //27
const int BIN2 = 14;  //14

const int STBY = 12; // Chân kích hoạt TB6612

// KHAI BÁO CHÂN ENCODER
const int ENC_L_A = 18;    //left
const int ENC_L_B = 19; 

const int ENC_R_A = 5;    //right
const int ENC_R_B = 4; 

// Biến đếm xung
volatile long countLeft = 0;
volatile long countRight = 0;

// --- HÀM NGẮT ĐỌC ENCODER TRÁI ---
void IRAM_ATTR isrEncoderLeft() {
  if (digitalRead(ENC_L_B) == HIGH) {
    countLeft++;
  } else {
    countLeft--;
  }
}

// --- HÀM NGẮT ĐỌC ENCODER PHẢI ---
void IRAM_ATTR isrEncoderRight() {
  if (digitalRead(ENC_R_B) == HIGH) {
    countRight++;
  } else {
    countRight--;
  }
}

void setup() {

  Serial.begin(115200);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // Kéo STBY lên HIGH để bật mạch TB6612

  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  
  //  bật trở kéo nội
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);

  // Cài đặt ngắt ngoài cho Encoder
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncoderRight, RISING);

  Serial.println("=== BẮT ĐẦU KIỂM TRA ĐỘNG CƠ VÀ ENCODER ===");
}

void loop() {

  Serial.println("\n[1] CHẠY TIẾN (Tốc độ 180/255)...");
  
  // Motor Trái (Kênh A) chạy tiến
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 180);

  // Motor Phải (Kênh B) chạy tiến
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, 180);

  delay(2000); // Chạy trong 2 giây

  Serial.print("-> Encoder Trái: ");
  Serial.print(countLeft);
  Serial.print(" | Encoder Phải: ");
  Serial.println(countRight);

  Serial.println("[2] DỪNG ĐỘNG CƠ...");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  delay(1000);


  Serial.println("[3] CHẠY LÙI (Tốc độ 180/255)...");
  
  // Motor Trái (Kênh A) chạy lùi
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 180);

  // Motor Phải (Kênh B) chạy lùi
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 180);

  delay(2000); // Chạy trong 2 giây

  // In giá trị xung đọc được
  Serial.print("-> Encoder Trái: ");
  Serial.print(countLeft);
  Serial.print(" | Encoder Phải: ");
  Serial.println(countRight);

  // ----------------------------------------------------
  // 4. DỪNG ĐỘNG CƠ (STOP)
  // ----------------------------------------------------
  Serial.println("[4] DỪNG ĐỘNG CƠ...");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  delay(1000);
}