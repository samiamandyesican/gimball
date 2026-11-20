#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <AccelStepper.h>
#include "esp_camera.h"

// ============================================================================
// 1. PIN ASSIGNMENTS (Conflict-Free for AI-Thinker ESP32-CAM without SD Card)
// ============================================================================
constexpr int PIN_I2C_SDA     = 13;
constexpr int PIN_I2C_SCL     = 14;
constexpr int PIN_SERVO_PITCH = 15;
constexpr int PIN_SERVO_ROLL  = 2;   // Must remain floating/LOW during firmware upload
constexpr int PIN_STEP_PULSE  = 12;  // Must NOT be pulled HIGH externally at boot
constexpr int PIN_STEP_DIR    = 4;   // Also drives onboard white flash LED

// AI-Thinker OV2640 Camera Pins
constexpr int PWDN_GPIO_NUM  = 32;
constexpr int RESET_GPIO_NUM = -1;
constexpr int XCLK_GPIO_NUM  = 0;
constexpr int SIOD_GPIO_NUM  = 26;
constexpr int SIOC_GPIO_NUM  = 27;
constexpr int Y9_GPIO_NUM    = 35;
constexpr int Y8_GPIO_NUM    = 34;
constexpr int Y7_GPIO_NUM    = 39;
constexpr int Y6_GPIO_NUM    = 36;
constexpr int Y5_GPIO_NUM    = 21;
constexpr int Y4_GPIO_NUM    = 19;
constexpr int Y3_GPIO_NUM    = 18;
constexpr int Y2_GPIO_NUM    = 5;
constexpr int VSYNC_GPIO_NUM = 25;
constexpr int HREF_GPIO_NUM  = 23;
constexpr int PCLK_GPIO_NUM  = 22;

// ============================================================================
// 2. CONFIG: TUNING & MECHANICAL PARAMETERS
// ============================================================================
constexpr bool ENABLE_CAMERA     = false;   // Set true once motor stabilization is verified
constexpr uint8_t L3G4200D_ADDR  = 0x69;    // Default I2C address (0x68 if SDO tied to GND)
constexpr float GYRO_SENSITIVITY = 0.0175f; // dps/digit at 500 dps full-scale range
constexpr float GYRO_DEADBAND    = 1.5f;    // Ignore noise below 1.5 deg/sec
constexpr float GYRO_DECAY       = 0.995f;  // Prevents unbounded gyro drift (0.98 - 0.999)

// Servo direction (+1.0 or -1.0 to invert) and neutral trim (degrees)
constexpr float PITCH_SIGN = 1.0f;
constexpr float ROLL_SIGN  = 1.0f;
constexpr float YAW_SIGN   = 1.0f;
constexpr int   PITCH_TRIM = 90;
constexpr int   ROLL_TRIM  = 90;

// Stepper scaling: (steps_per_rev * microstepping * gear_ratio) / 360.0
// Example: 200 steps/rev * 16 microsteps / 360 deg = 8.8889 steps/deg
constexpr float STEPS_PER_DEGREE = 8.8889f;

// ============================================================================
// 3. GLOBAL OBJECTS & STATE
// ============================================================================
Servo servoPitch;
Servo servoRoll;
AccelStepper yawStepper(AccelStepper::DRIVER, PIN_STEP_PULSE, PIN_STEP_DIR);

float biasX = 0.0f, biasY = 0.0f, biasZ = 0.0f;
float anglePitch = 0.0f, angleRoll = 0.0f, angleYaw = 0.0f;
unsigned long lastMicros = 0;

// ============================================================================
// 4. L3G4200D I2C DRIVER FUNCTIONS
// ============================================================================
void writeGyroReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(L3G4200D_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool initGyro() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

  // Verify WHO_AM_I register (0x0F should return 0xD3)
  Wire.beginTransmission(L3G4200D_ADDR);
  Wire.write(0x0F);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(L3G4200D_ADDR, (uint8_t)1);
  if (!Wire.available() || Wire.read() != 0xD3) return false;

  // CTRL_REG1 (0x20): 200 Hz ODR, 50 Hz cutoff, Normal mode, X/Y/Z enabled (0b01101111)
  writeGyroReg(0x20, 0x6F);
  // CTRL_REG4 (0x23): Block Data Update enabled, 500 dps full-scale (0b10010000)
  writeGyroReg(0x23, 0x90);
  return true;
}

void readGyroRaw(int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(L3G4200D_ADDR);
  Wire.write(0x28 | 0x80); // OUT_X_L with auto-increment bit (MSB = 1)
  Wire.endTransmission(false);
  Wire.requestFrom(L3G4200D_ADDR, (uint8_t)6);

  if (Wire.available() == 6) {
    uint8_t xlo = Wire.read(), xhi = Wire.read();
    uint8_t ylo = Wire.read(), yhi = Wire.read();
    uint8_t zlo = Wire.read(), zhi = Wire.read();
    gx = (int16_t)(xhi << 8 | xlo);
    gy = (int16_t)(yhi << 8 | ylo);
    gz = (int16_t)(zhi << 8 | zlo);
  }
}

void calibrateGyro(int samples = 400) {
  Serial.println("Calibrating L3G4200D - Keep gimbal stationary...");
  long sumX = 0, sumY = 0, sumZ = 0;
  int16_t gx, gy, gz;

  for (int i = 0; i < samples; i++) {
    readGyroRaw(gx, gy, gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(5);
  }
  biasX = (float)sumX / samples;
  biasY = (float)sumY / samples;
  biasZ = (float)sumZ / samples;
  Serial.println("Calibration complete.");
}

// ============================================================================
// 5. CAMERA INITIALIZATION (Preserves LEDC Timers 1-3 for Servos)
// ============================================================================
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0; // Must use Timer 0 / Channel 0
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_VGA;
  config.jpeg_quality = 12;
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  return (esp_camera_init(&config) == ESP_OK);
}

// ============================================================================
// 6. SETUP & CONTROL LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Reserve LEDC Timers 1-3 for servos BEFORE initializing the camera (Timer 0)
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoPitch.setPeriodHertz(50);
  servoRoll.setPeriodHertz(50);
  servoPitch.attach(PIN_SERVO_PITCH, 500, 2400);
  servoRoll.attach(PIN_SERVO_ROLL, 500, 2400);
  servoPitch.write(PITCH_TRIM);
  servoRoll.write(ROLL_TRIM);

  yawStepper.setMaxSpeed(2000.0f);     // steps/sec
  yawStepper.setAcceleration(8000.0f); // steps/sec^2

  if (ENABLE_CAMERA) {
    if (!initCamera()) {
      Serial.println("ERROR: Camera initialization failed.");
    } else {
      Serial.println("Camera initialized.");
    }
  }

  if (!initGyro()) {
    Serial.println("ERROR: L3G4200D not detected on SDA=13, SCL=14. Check wiring/address.");
    while (true) { delay(100); }
  }

  calibrateGyro();
  lastMicros = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;

  // Run sensor integration and servo updates at ~200 Hz (every 5000 us)
  if (dt >= 0.005f) {
    lastMicros = now;

    int16_t rawX, rawY, rawZ;
    readGyroRaw(rawX, rawY, rawZ);

    // Convert raw readings to degrees/sec
    float rateX = (rawX - biasX) * GYRO_SENSITIVITY;
    float rateY = (rawY - biasY) * GYRO_SENSITIVITY;
    float rateZ = (rawZ - biasZ) * GYRO_SENSITIVITY;

    // Apply deadband to suppress stationary noise
    if (fabs(rateX) < GYRO_DEADBAND) rateX = 0.0f;
    if (fabs(rateY) < GYRO_DEADBAND) rateY = 0.0f;
    if (fabs(rateZ) < GYRO_DEADBAND) rateZ = 0.0f;

    // Integrate angular rates and apply gentle drift decay toward center
    anglePitch = (anglePitch + rateX * dt) * GYRO_DECAY;
    angleRoll  = (angleRoll  + rateY * dt) * GYRO_DECAY;
    angleYaw   = (angleYaw   + rateZ * dt) * GYRO_DECAY;

    // Apply compensating angles to servos (clamped to 0 - 180 deg)
    int cmdPitch = constrain((int)(PITCH_TRIM - PITCH_SIGN * anglePitch), 0, 180);
    int cmdRoll  = constrain((int)(ROLL_TRIM  - ROLL_SIGN  * angleRoll),  0, 180);
    servoPitch.write(cmdPitch);
    servoRoll.write(cmdRoll);

    // Command target step position for yaw stepper
    long targetSteps = (long)(-YAW_SIGN * angleYaw * STEPS_PER_DEGREE);
    yawStepper.moveTo(targetSteps);
  }

  // Must be called as fast as possible for smooth stepper pulse generation
  yawStepper.run();
}