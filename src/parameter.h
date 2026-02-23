#pragma once

// ======================================
// Boot + UART parameters
// ======================================
static constexpr bool BOOT_MOTOR_CYCLE = true;
static constexpr bool ENABLE_UART_TRANSPORT = false;
static constexpr uint32_t READY_FAULT_TIMEOUT_MS = 3000;
// Cooldown after a full cycle completes
static constexpr uint32_t STEPPER_COOLDOWN_MS = 8000;

// ======================================
// AI inference parameters
// ======================================
static constexpr uint8_t CONFIDENCE_THRESHOLD = 70; // percent
// Number of consecutive detections required to trigger any actuation
static constexpr uint8_t ACTUATION_REQUIRED_OCCURRENCES = 2;

// ======================================
// Motor timing + rotation parameters
// ======================================
// Adjust these values to tune the stepper.
// Future: these will be set via DIP switches.

// 28BYJ-48 in bipolar mode often uses 2048 or 4096 steps/rev depending on gearbox.
static constexpr uint32_t STEPPER_STEPS_PER_REV = 2048;

// 400 steps/s -> 2500 us per step
static constexpr uint32_t STEPPER_STEP_INTERVAL_US = 2500;

// Pause between forward and reverse cycles
static constexpr uint32_t STEPPER_POST_MOVE_PAUSE_MS = 900;

// Motor drive strength (1..63)
static constexpr uint8_t STEPPER_VSET = 40;

// ======================================
// WiFi / Web parameters
// ======================================
// WEB_MODE: 0 = disabled, 1 = client (STA), 2 = access point (AP)
static constexpr uint8_t WEB_MODE = 2;
static constexpr const char *WEB_SSID = "VST-ONE";
static constexpr const char *WEB_PASSWORD = "";
// AP-specific SSID options
static constexpr const char *WEB_AP_BASE = "VST-ONE";
static constexpr bool WEB_AP_APPEND_MAC = true;

// ======================================
// LED timing + class mapping
// ======================================
static constexpr uint32_t LED_ON_MS = 2000;

// LED GPIOs (XIAO ESP32-S3)
static constexpr int LED_D0 = 1; // D0, PIN 1, RED  (Grove LED / Relay)
static constexpr int LED_D1 = 2; // D1, PIN 2, GREEN
static constexpr int LED_D2 = 3; // D2, PIN 3, WHITE

// AI class -> LED mapping
static constexpr uint8_t CLASS_LED1 = 3; // Vespa velutina
static constexpr uint8_t CLASS_LED2 = 2; // Vespula spp.
static constexpr uint8_t CLASS_LED3 = 1; // Vespa crabro

// ======================================
// DIP switch mapping (6-bit -> 2+2+2)
// bits 1:0 = direction, 3:2 = angle, 5:4 = wait
// ======================================
static constexpr uint8_t DIP_DIR_CCW_ONLY = 0;
static constexpr uint8_t DIP_DIR_CW_ONLY = 1;
static constexpr uint8_t DIP_DIR_CCW_THEN_CW = 2;
static constexpr uint8_t DIP_DIR_CW_THEN_CCW = 3;

// Direction mapping for DIP bits 1:0:
// 00 -> CCW only
// 01 -> CCW then CW
// 10 -> CW only
// 11 -> CW then CCW
static constexpr uint8_t DIP_DIR_MAP[4] = {
    // index = (switch2<<1 | switch1)
    DIP_DIR_CCW_ONLY,      // 00
    DIP_DIR_CW_ONLY,       // 01
    DIP_DIR_CCW_THEN_CW,   // 10
    DIP_DIR_CW_THEN_CCW,   // 11
};

static constexpr uint16_t DIP_ANGLE_DEG[4] = { 90, 180, 360, 720 };
static constexpr uint32_t DIP_WAIT_MS[4] = { 0, 6000, 30000, 60000 };

// Default motor behavior when DIP switch is not detected
static constexpr uint8_t DEFAULT_DIR_MODE = DIP_DIR_CW_THEN_CCW;
static constexpr uint16_t DEFAULT_ANGLE_DEG = 720;
static constexpr uint32_t DEFAULT_WAIT_MS = 60000;
