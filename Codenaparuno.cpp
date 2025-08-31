//================================================================
// Tên file: arduino_firmware_freertos.ino
// Mô tả: Phiên bản nâng cấp sử dụng FreeRTOS để quản lý đa nhiệm,
//        giúp robot phản ứng nhanh và hoạt động ổn định hơn.
//================================================================

#include <Arduino_FreeRTOS.h> // Thư viện chính của FreeRTOS
#include <AccelStepper.h>
#include <Servo.h>
#include <semphr.h> // Để sử dụng Mutex/Semaphore nếu cần sau này

// --- CÀI ĐẶT GIAO TIẾP VÀ GIAO THỨC (Không đổi) ---
#define BAUD 115200
#define HEADER1 0xAA
#define HEADER2 0x55
#define MAX_BUFFER 64

// --- ĐỊNH NGHĨA CÁC CHÂN (PIN) (Không đổi) ---
#define X_LIMIT_PIN 9
#define Y_LIMIT_PIN 10
#define Z_LIMIT_PIN 11
#define ENABLE_PIN 8
#define SERVO_PIN A1
#define ServoOpen 90
#define ServoClose 45
#define X_STEP_PIN 2
#define X_DIR_PIN 5
#define Y_STEP_PIN 3
#define Y_DIR_PIN 6
#define Z_STEP_PIN 4
#define Z_DIR_PIN 7

// --- CẤU HÌNH THÔNG SỐ ĐỘNG CƠ (Không đổi) ---
#define MOTOR_INTERFACE_TYPE 1
#define MICROSTEPS 16
#define STEPS_PER_REV 200
#define GEAR_RATIO 4.5
#define X_MAX_SPEED 5000.0
#define X_ACCELERATION 2000.0
#define X_HOMING_SPEED -1500
#define Y_MAX_SPEED 5000.0
#define Y_ACCELERATION 2000.0
#define Y_HOMING_SPEED -1500
#define Z_MAX_SPEED 5000.0
#define Z_ACCELERATION 2000.0
#define Z_HOMING_SPEED -1500.0

// --- BIẾN TOÀN CỤC VÀ ĐỐI TƯỢNG ---
enum RobotStatus { IDLE, HOMING, RUNNING, EMERGENCY };
// Sử dụng 'volatile' vì biến này được chia sẻ giữa các task
volatile RobotStatus robot_status = IDLE;

Servo GripperServo;
AccelStepper stepperX(MOTOR_INTERFACE_TYPE, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(MOTOR_INTERFACE_TYPE, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ(MOTOR_INTERFACE_TYPE, Z_STEP_PIN, Z_DIR_PIN);

// --- KHAI BÁO NGUYÊN MẪU HÀM VÀ TASK ---
void processPacket(byte *packet, byte len);
void sendStatusResponse(byte cmd, byte option, uint16_t transition, const byte* payload = nullptr, byte payload_len = 0);
void setTargetAngles(float angleX, float angleY, float angleZ);
void home_move_XYZ(long j1, long j2, long j3);

// Khai báo các Task cho FreeRTOS
void Task_SerialCommander(void *pvParameters);
void Task_StepperRunner(void *pvParameters);

// --- HÀM KHỞI TẠO (SETUP) ---
void setup() {
  Serial.begin(BAUD);

  // Khởi tạo phần cứng
  pinMode(X_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Y_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Z_LIMIT_PIN, INPUT_PULLUP);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // LOW = enable drivers

  GripperServo.attach(SERVO_PIN);
  GripperServo.write(ServoOpen);

  // Cấu hình các động cơ
  stepperX.setMaxSpeed(X_MAX_SPEED);
  stepperX.setAcceleration(X_ACCELERATION);
  stepperY.setMaxSpeed(Y_MAX_SPEED);
  stepperY.setAcceleration(Y_ACCELERATION);
  stepperZ.setMaxSpeed(Z_MAX_SPEED);
  stepperZ.setAcceleration(Z_ACCELERATION);

  // Khởi tạo các Task
  xTaskCreate(
    Task_SerialCommander,   // Hàm của task
    "SerialCommander",      // Tên của task
    512,                    // Kích thước stack (bytes)
    NULL,                   // Tham số truyền vào task
    2,                      // Độ ưu tiên (cao hơn)
    NULL);                  // Con trỏ handle của task

  xTaskCreate(
    Task_StepperRunner,
    "StepperRunner",
    128,
    NULL,
    1,                      // Độ ưu tiên (thấp hơn)
    NULL);
    
  // Bộ lập lịch FreeRTOS sẽ bắt đầu chạy sau khi setup() kết thúc
}

// --- VÒNG LẶP CHÍNH (LOOP) ---
// Sẽ không bao giờ được chạy vì FreeRTOS đã chiếm quyền điều khiển
void loop() {}

// --- ĐỊNH NGHĨA CÁC TASK ---

/**
 * @brief Task này chịu trách nhiệm lắng nghe và xử lý các lệnh từ Serial.
 */
void Task_SerialCommander(void *pvParameters) {
  (void) pvParameters; // Tránh warning không sử dụng tham số
  
  static byte buffer[MAX_BUFFER];
  static byte index = 0;
  static byte expected_len = 0;

  for (;;) { // Vòng lặp vô tận của Task
    while (Serial.available()) {
      byte b = Serial.read();
      if (index == 0 && b != HEADER1) continue;
      if (index == 1 && b != HEADER2) { index = 0; continue; }
      buffer[index++] = b;

      if (index == 3) {
        expected_len = buffer[2];
        if (expected_len > (MAX_BUFFER - 2)) {
          index = 0; // Lỗi gói tin, reset
        }
      }

      // Nếu đã nhận đủ gói tin
      if (index > 2 && index == expected_len + 3) {
        processPacket(buffer, index);
        index = 0; // Reset để nhận gói tin tiếp theo
      }
    }
    
    // Tạm dừng task 10ms để các task khác (và hệ thống) có thời gian chạy
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Task này chạy nền liên tục để điều khiển chuyển động của các stepper.
 */
void Task_StepperRunner(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    // Chỉ chạy động cơ khi robot ở trạng thái RUNNING
    if (robot_status == RUNNING) {
      stepperX.run();
      stepperY.run();
      stepperZ.run();
    }
    
    // Tạm dừng 1ms. Điều này cực kỳ quan trọng để không làm treo CPU
    // và đảm bảo các task khác có thể chạy.
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}


// --- CÁC HÀM XỬ LÝ GÓI TIN (Không đổi) ---
void processPacket(byte *packet, byte len) {
  byte cmd = packet[3];
  byte option = packet[4];
  uint16_t transition = (packet[5] << 8) | packet[6];
  byte payload_len = len - 8;
  byte *payload = &packet[7];
  byte received_crc = packet[len - 1];

  byte calc_crc = 0;
  for (int i = 3; i < len - 1; i++) { calc_crc ^= packet[i]; }
  if (calc_crc != received_crc) { return; } // Bỏ qua nếu CRC sai

  switch (cmd) {
    case 0x02: // Lệnh dừng khẩn cấp
      if (option == 0x00) {
        digitalWrite(ENABLE_PIN, HIGH); // HIGH = disable drivers
        robot_status = EMERGENCY;
        sendStatusResponse(cmd, 0x01, transition);
      }
      break;

    case 0x03: // Lệnh về Home
      if (option == 0x00 && payload_len >= 6) {
        digitalWrite(ENABLE_PIN, LOW);
        long j1 = (payload[0] << 8) | payload[1];
        long j2 = (payload[2] << 8) | payload[3];
        long j3 = (payload[4] << 8) | payload[5];
        robot_status = HOMING;
        sendStatusResponse(cmd, 0x02, transition, payload, payload_len);
        
        // Hàm này sẽ "chặn" Task_SerialCommander cho đến khi xong,
        // nhưng các task khác vẫn có thể chạy.
        home_move_XYZ(j1, j2, j3);
        
        robot_status = IDLE;
      }
      break;

    case 0x04: // Lệnh điều khiển Gripper
      if (option == 0x00 && payload_len >= 1) {
        GripperServo.write(payload[0] ? ServoOpen : ServoClose);
        sendStatusResponse(cmd, 0x01, transition);
      }
      break;

    case 0x05: // Lệnh di chuyển tới góc
      if (option == 0x00 && payload_len >= 12) {
        // Đảm bảo robot đã được enable
        digitalWrite(ENABLE_PIN, LOW);
        
        float joint1, joint2, joint3;
        memcpy(&joint1, &payload[0], 4);
        memcpy(&joint2, &payload[4], 4);
        memcpy(&joint3, &payload[8], 4);

        setTargetAngles(joint3, joint2, joint1);
        robot_status = RUNNING; // Chỉ đặt trạng thái RUNNING sau khi đã có mục tiêu
        sendStatusResponse(cmd, 0x01, transition);
      }
      break;
  }
}

// --- CÁC HÀM TIỆN ÍCH (Không đổi) ---
void sendStatusResponse(byte cmd, byte option, uint16_t transition, const byte* payload, byte payload_len) {
  byte response_len = 1 + 1 + 2 + payload_len;
  byte total_len = 2 + 1 + response_len + 1;
  if (total_len > MAX_BUFFER) return;
  static byte packet[MAX_BUFFER];
  packet[0] = HEADER1; packet[1] = HEADER2; packet[2] = response_len;
  packet[3] = cmd; packet[4] = option;
  packet[5] = (transition >> 8) & 0xFF; packet[6] = transition & 0xFF;
  if (payload_len > 0 && payload != nullptr) { memcpy(&packet[7], payload, payload_len); }
  byte crc = 0;
  for (int i = 3; i < 7 + payload_len; i++) { crc ^= packet[i]; }
  packet[7 + payload_len] = crc;
  Serial.write(packet, total_len);
  Serial.flush();
}

void setTargetAngles(float angleX, float angleY, float angleZ) {
  long Xsteps = (angleX * MICROSTEPS * STEPS_PER_REV * GEAR_RATIO) / 360.0;
  long Ysteps = (angleY * MICROSTEPS * STEPS_PER_REV * GEAR_RATIO) / 360.0;
  long Zsteps = (angleZ * MICROSTEPS * STEPS_PER_REV * GEAR_RATIO) / 360.0;
  stepperX.moveTo(Xsteps);
  stepperY.moveTo(Ysteps);
  stepperZ.moveTo(Zsteps);
}

void home_move_XYZ(long j1, long j2, long j3) {
  stepperX.setSpeed(X_HOMING_SPEED); 
  stepperY.setSpeed(Y_HOMING_SPEED); 
  stepperZ.setSpeed(Z_HOMING_SPEED);
  
  bool xHomed = false, yHomed = false, zHomed = false;
  
  while (!xHomed || !yHomed || !zHomed) {
    if (!xHomed) { 
      if (digitalRead(X_LIMIT_PIN) != LOW) {
        stepperX.runSpeed(); 
      } else { 
        stepperX.stop(); stepperX.setCurrentPosition(0); xHomed = true; 
      } 
    }
    if (!yHomed) { 
      if (digitalRead(Y_LIMIT_PIN) != LOW) {
        stepperY.runSpeed(); 
      } else { 
        stepperY.stop(); stepperY.setCurrentPosition(0); yHomed = true; 
      } 
    }
    if (!zHomed) { 
      if (digitalRead(Z_LIMIT_PIN) != LOW) {
        stepperZ.runSpeed(); 
      } else { 
        stepperZ.stop(); stepperZ.setCurrentPosition(0); zHomed = true; 
      } 
    }
    vTaskDelay(1 / portTICK_PERIOD_MS); // Yield cho các task khác nếu cần
  }
  
  stepperX.moveTo(j1); 
  stepperY.moveTo(j2); 
  stepperZ.moveTo(j3);
  
  while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0 || stepperZ.distanceToGo() != 0) {
    stepperX.run(); stepperY.run(); stepperZ.run();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  
  stepperX.setCurrentPosition(0); 
  stepperY.setCurrentPosition(0); 
  stepperZ.setCurrentPosition(0);
}