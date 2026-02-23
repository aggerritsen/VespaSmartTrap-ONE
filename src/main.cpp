// main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <Seeed_Arduino_SSCMA.h>
#include <esp_heap_caps.h>
#include "esp_crc.h"
#include "esp_timer.h"
#include "parameter.h"
#include "web.h"

/* ================================
   OLED (XIAO Expansion Board)
   ================================ */
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Grove_Multi_Switch.h"

static constexpr uint8_t OLED_ADDR = 0x3C;
static constexpr int OLED_W = 128;
static constexpr int OLED_H = 64;

Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
static bool OLED_OK = false;

static uint8_t  oled_last_target = 255;
static uint8_t  oled_last_score  = 255;
static uint32_t oled_last_ms     = 0;
static uint32_t oled_boot_last_ms = 0;
static bool     oled_have_detection = false;

SSCMA AI;

static constexpr uint8_t VISION_AI_ADDR = 0x62;

/* ================================
   MINI MOTOR DRIVER (DRV8830 x2)
   ================================ */
static constexpr bool ENABLE_STEPPER = true;

// I2C addresses for the two DRV8830s (7-bit)
static constexpr uint8_t COIL_A_ADDR = 0x65;
static constexpr uint8_t COIL_B_ADDR = 0x60;

// Optional fault pin from DRV8830 (active LOW). Set to -1 if not connected.
static constexpr int STEPPER_FAULT_PIN = -1;

// DRV8830 registers
static constexpr uint8_t REG_CONTROL = 0x00;
static constexpr uint8_t REG_FAULT = 0x01;

// Fault bits
static constexpr uint8_t FAULT = 0x01;
static constexpr uint8_t OCP = 0x02;
static constexpr uint8_t UVLO = 0x04;
static constexpr uint8_t OTS = 0x08;
static constexpr uint8_t ILIMIT = 0x10;

class MiniMoto
{
public:
    explicit MiniMoto(uint8_t addr7)
        : _addr(addr7)
    {
    }

    void drive(int speed)
    {
        // speed: -63..63 (sign = direction)
        int v = speed;
        if (v > 63) v = 63;
        if (v < -63) v = -63;

        if (v == 0)
        {
            stop();
            return;
        }

        uint8_t vset = (uint8_t)abs(v);
        uint8_t in1 = (v > 0) ? 1 : 0;
        uint8_t in2 = (v > 0) ? 0 : 1;
        writeControl(vset, in1, in2);
    }

    void stop()
    {
        // Coast
        writeControl(0, 0, 0);
    }

    void brake()
    {
        // Brake (short)
        writeControl(0, 1, 1);
    }

    uint8_t getFault()
    {
        return readReg(REG_FAULT);
    }

private:
    uint8_t _addr;

    void writeControl(uint8_t vset, uint8_t in1, uint8_t in2)
    {
        uint8_t ctrl = (uint8_t)((vset & 0x3F) << 2) | ((in2 & 0x01) << 1) | (in1 & 0x01);
        writeReg(REG_CONTROL, ctrl);
    }

    void writeReg(uint8_t reg, uint8_t val)
    {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }

    uint8_t readReg(uint8_t reg)
    {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom((int)_addr, 1);
        if (Wire.available())
        {
            return (uint8_t)Wire.read();
        }
        return 0;
    }
};

static MiniMoto coilA(COIL_A_ADDR);
static MiniMoto coilB(COIL_B_ADDR);

enum Dir : int8_t { COAST = 0, FWD = +1, REV = -1 };

static void coilSet(MiniMoto &coil, Dir d, uint8_t vset)
{
    if (d == FWD) coil.drive((int)vset);
    else if (d == REV) coil.drive(-(int)vset);
    else coil.stop();
}

static void printFault(const char *label, byte result)
{
    if (result & FAULT)
    {
        Serial.print(label);
        if (result & OCP) Serial.print(" Overcurrent");
        if (result & ILIMIT) Serial.print(" Current limit");
        if (result & UVLO) Serial.print(" Undervoltage");
        if (result & OTS) Serial.print(" Overtemp");
        Serial.println();
    }
}

static bool pollStepperFaults()
{
    bool faulted = false;
    byte result0 = coilA.getFault();
    if (result0 & FAULT)
    {
        printFault("Coil A fault:", result0);
        faulted = true;
    }

    byte result1 = coilB.getFault();
    if (result1 & FAULT)
    {
        printFault("Coil B fault:", result1);
        faulted = true;
    }

    return faulted;
}

static void stepperRelease()
{
    coilSet(coilA, COAST, 0);
    coilSet(coilB, COAST, 0);
}

static void stepperStepFull(uint32_t idx, uint8_t vset)
{
    switch (idx & 0x03)
    {
        case 0: coilSet(coilA, FWD, vset); coilSet(coilB, FWD, vset); break; // A+, B+
        case 1: coilSet(coilA, REV, vset); coilSet(coilB, FWD, vset); break; // A-, B+
        case 2: coilSet(coilA, REV, vset); coilSet(coilB, REV, vset); break; // A-, B-
        case 3: coilSet(coilA, FWD, vset); coilSet(coilB, REV, vset); break; // A+, B-
    }
}

enum StepperPhase : uint8_t
{
    STEPPER_FWD = 0,
    STEPPER_PAUSE_FWD = 1,
    STEPPER_REV = 2,
    STEPPER_PAUSE_REV = 3,
};

struct StepperState
{
    bool enabled = false;
    bool run_once = true;
    bool active = false;
    bool present = false;
    uint32_t last_trigger_ms = 0;
    uint32_t last_cycle_end_ms = 0;
    uint8_t direction_mode = DIP_DIR_CCW_THEN_CW;
    uint32_t wait_ms = 0;
    uint32_t steps_per_move = 0;
    bool last_dir_rev = false;
    StepperPhase phase = STEPPER_FWD;
    uint32_t steps_per_rev = STEPPER_STEPS_PER_REV;
    uint32_t step_interval_us = STEPPER_STEP_INTERVAL_US;
    uint32_t post_move_pause_ms = STEPPER_POST_MOVE_PAUSE_MS;
    uint8_t vset = STEPPER_VSET;

    uint32_t step_index = 0;
    uint32_t next_step_us = 0;
    uint32_t pause_until_ms = 0;
    uint32_t last_fault_poll_ms = 0;
};

static StepperState stepper;

static void apply_default_stepper()
{
    stepper.direction_mode = DEFAULT_DIR_MODE;
    stepper.wait_ms = DEFAULT_WAIT_MS;
    uint32_t steps = (stepper.steps_per_rev * (uint32_t)DEFAULT_ANGLE_DEG) / 360U;
    if (steps == 0) steps = 1;
    stepper.steps_per_move = steps;
}

static bool stepperDetectI2C()
{
    // Probe both DRV8830 addresses. Require both for stepper operation.
    auto probe = [](uint8_t addr) -> bool {
        Wire.beginTransmission(addr);
        Wire.write(REG_FAULT);
        if (Wire.endTransmission() != 0)
            return false;
        Wire.requestFrom((int)addr, 1);
        if (Wire.available())
        {
            (void)Wire.read();
            return true;
        }
        return false;
    };

    return probe(COIL_A_ADDR) && probe(COIL_B_ADDR);
}

static bool visionAiProbeI2C()
{
    Wire.beginTransmission(VISION_AI_ADDR);
    return (Wire.endTransmission() == 0);
}

static inline bool stepperInCooldown()
{
    if (!ENABLE_STEPPER)
        return false;
    if (stepper.active)
        return false;
    uint32_t now = millis();
    return (now - stepper.last_cycle_end_ms) < STEPPER_COOLDOWN_MS;
}

static void stepperInit()
{
    if (!ENABLE_STEPPER)
        return;

    stepper.run_once = true;
    stepper.active = false;
    stepper.present = stepperDetectI2C();
    stepper.phase = STEPPER_FWD;
    stepper.step_index = 0;
    stepper.next_step_us = micros();
    stepper.pause_until_ms = 0;
    stepper.last_fault_poll_ms = 0;
    stepper.last_trigger_ms = 0;
    stepper.last_cycle_end_ms = 0;
    apply_default_stepper();

    if (STEPPER_FAULT_PIN >= 0)
    {
        pinMode(STEPPER_FAULT_PIN, INPUT_PULLUP);
    }

    stepper.enabled = false;
    if (stepper.present)
        Serial.println("✅ Stepper ready (triggered on class 3)");
    else
        Serial.println("⚠️ Stepper driver not detected (motor disabled)");
}

static void stepperStartCycle(bool force = false)
{
    if (!ENABLE_STEPPER)
        return;

    if (!stepper.present)
        return;

    if (stepper.active)
        return;

    uint32_t now = millis();
    if (!force && (now - stepper.last_cycle_end_ms) < STEPPER_COOLDOWN_MS)
        return;

    stepper.enabled = true;
    stepper.active = true;
    stepper.last_trigger_ms = now;
    stepper.phase = (stepper.direction_mode == DIP_DIR_CCW_ONLY || stepper.direction_mode == DIP_DIR_CCW_THEN_CW)
        ? STEPPER_REV
        : STEPPER_FWD;
    stepper.step_index = 0;
    stepper.next_step_us = micros();
    stepper.pause_until_ms = 0;
    stepper.last_fault_poll_ms = 0;
}

static inline void trigger_motor_cycle(bool force = false)
{
    stepperStartCycle(force);
}

static void stepperUpdate()
{
    if (!stepper.enabled)
        return;

    // Fault polling (avoid spamming I2C)
    uint32_t now_ms = millis();
    if (STEPPER_FAULT_PIN >= 0)
    {
        if (digitalRead(STEPPER_FAULT_PIN) == LOW)
        {
            pollStepperFaults();
        }
    }
    else if (now_ms - stepper.last_fault_poll_ms >= 50)
    {
        stepper.last_fault_poll_ms = now_ms;
        pollStepperFaults();
    }

    switch (stepper.phase)
    {
        case STEPPER_FWD:
        case STEPPER_REV:
        {
            uint32_t now_us = micros();
            int32_t delta = (int32_t)(now_us - stepper.next_step_us);
            if (delta < 0)
                return;

            uint32_t idx = stepper.step_index;
            bool reverse = (stepper.phase == STEPPER_REV);
            stepper.last_dir_rev = reverse;
            uint32_t seq = reverse ? (stepper.steps_per_move - 1 - idx) : idx;
            stepperStepFull(seq, stepper.vset);

            stepper.step_index++;
            // Mirror the example's fixed delay between steps.
            stepper.next_step_us = now_us + stepper.step_interval_us;

            if (stepper.step_index >= stepper.steps_per_move)
            {
                stepperRelease();
                stepper.step_index = 0;
                bool two_dir = (stepper.direction_mode == DIP_DIR_CCW_THEN_CW ||
                                stepper.direction_mode == DIP_DIR_CW_THEN_CCW);
                bool between_legs = false;
                if (two_dir)
                {
                    // Wait only between the two legs
                    between_legs =
                        (stepper.direction_mode == DIP_DIR_CW_THEN_CCW && stepper.phase == STEPPER_FWD) ||
                        (stepper.direction_mode == DIP_DIR_CCW_THEN_CW && stepper.phase == STEPPER_REV);
                }

                stepper.pause_until_ms = between_legs ? (now_ms + stepper.wait_ms) : now_ms;
                stepper.phase = (stepper.phase == STEPPER_FWD) ? STEPPER_PAUSE_FWD : STEPPER_PAUSE_REV;
            }
            break;
        }
        case STEPPER_PAUSE_FWD:
        case STEPPER_PAUSE_REV:
        {
            if ((int32_t)(now_ms - stepper.pause_until_ms) >= 0)
            {
                stepper.next_step_us = micros();
                if (stepper.phase == STEPPER_PAUSE_FWD)
                {
                    if (stepper.direction_mode == DIP_DIR_CW_ONLY)
                    {
                        stepperRelease();
                        stepper.enabled = false;
                        stepper.active = false;
                        stepper.last_cycle_end_ms = millis();
                        Serial.println("✅ Stepper cycle done (run_once)");
                    }
                    else if (stepper.direction_mode == DIP_DIR_CW_THEN_CCW)
                    {
                        stepper.phase = STEPPER_REV;
                    }
                    else if (stepper.direction_mode == DIP_DIR_CCW_THEN_CW)
                    {
                        stepperRelease();
                        stepper.enabled = false;
                        stepper.active = false;
                        stepper.last_cycle_end_ms = millis();
                        Serial.println("✅ Stepper cycle done (run_once)");
                    }
                    else
                    {
                        stepper.phase = STEPPER_REV;
                    }
                }
                else
                {
                    // Completed FWD + REV cycle
                    if (stepper.direction_mode == DIP_DIR_CCW_ONLY)
                    {
                        stepperRelease();
                        stepper.enabled = false;
                        stepper.active = false;
                        stepper.last_cycle_end_ms = millis();
                        Serial.println("✅ Stepper cycle done (run_once)");
                    }
                    else if (stepper.direction_mode == DIP_DIR_CCW_THEN_CW)
                    {
                        stepper.phase = STEPPER_FWD;
                    }
                    else if (stepper.direction_mode == DIP_DIR_CW_THEN_CCW)
                    {
                        stepperRelease();
                        stepper.enabled = false;
                        stepper.active = false;
                        stepper.last_cycle_end_ms = millis();
                        Serial.println("✅ Stepper cycle done (run_once)");
                    }
                    else if (stepper.run_once)
                    {
                        stepperRelease();
                        stepper.enabled = false;
                        stepper.active = false;
                        stepper.last_cycle_end_ms = millis();
                        Serial.println("✅ Stepper cycle done (run_once)");
                    }
                    else
                    {
                        stepper.phase = STEPPER_FWD;
                    }
                }
            }
            break;
        }
        default:
            break;
    }
}

/* ================================
   UART CONFIG (XIAO → T-SIM)
   ================================ */
static constexpr uint32_t UART_BAUD = 921600;
static constexpr int UART_TX_PIN = 43;
static constexpr int UART_RX_PIN = 44;

/* ================================
   ACTUATORS
   ================================ */
static constexpr int LED_PIN_1 = LED_D0;
static constexpr int LED_PIN_2 = LED_D1;
static constexpr int LED_PIN_3 = LED_D2;

static uint32_t led1_until = 0;
static uint32_t led2_until = 0;
static uint32_t led3_until = 0;

/* Actuator timers (more responsive than relying on loop timing) */
static esp_timer_handle_t led1_timer = nullptr;
static esp_timer_handle_t led2_timer = nullptr;
static esp_timer_handle_t led3_timer = nullptr;

static bool boot_test_started = false;
static bool boot_motor_pending = false;
static uint32_t boot_motor_deadline_ms = 0;
static uint32_t boot_end_ms = 0;
static constexpr uint32_t BOOT_STATUS_TIMEOUT_MS = READY_FAULT_TIMEOUT_MS;
static bool vision_ai_ok = false;
static uint32_t last_inference_ms = 0;

static inline bool bootTestActive()
{
    if (!boot_test_started)
        return false;

    if (stepper.active)
        return true;

    uint32_t now = millis();
    bool leds_active = (led1_until != 0 && (int32_t)(now - led1_until) < 0) ||
                       (led2_until != 0 && (int32_t)(now - led2_until) < 0) ||
                       (led3_until != 0 && (int32_t)(now - led3_until) < 0);
    if (!leds_active)
    {
        boot_test_started = false;
        return false;
    }

    return true;
}

static bool motorDirIsRev();
static bool stepperInPause();
static const char* motorLabelForOled();
static uint32_t oled_wait_remaining_seconds();

static void oled_show_boot()
{
    if (!OLED_OK) return;

    uint32_t now = millis();
    if ((now - oled_boot_last_ms) < 300)
        return;
    oled_boot_last_ms = now;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("Boot");

    display.setTextSize(1);
    display.setCursor(0, 22);
    display.println("test");

    if (stepper.active)
    {
        // Right-aligned direction indicator (match RUN style)
        const char *sym = motorLabelForOled();
        display.setTextSize(2);
        int16_t x1, y1;
        uint16_t w, h;
        display.getTextBounds(sym, 0, 0, &x1, &y1, &w, &h);
        int16_t x = (int16_t)(OLED_W - w);
        int16_t y = 2;
        if (x < 0) x = 0;
        display.setCursor(x, y);
        display.print(sym);
    }

    display.setTextSize(1);
    display.setCursor(0, 52);
    if (stepper.active &&
        stepperInPause() &&
        stepper.wait_ms > 0)
    {
        uint32_t rem = oled_wait_remaining_seconds();
        display.print("Wait ");
        display.print(rem);
        display.print("s");
    }
    else
    {
        display.print("Boot test");
    }

    display.display();
}

static void oled_show_ready_fault(bool vision_ok)
{
    if (!OLED_OK) return;

    uint32_t now = millis();
    if ((now - oled_boot_last_ms) < 500)
        return;
    oled_boot_last_ms = now;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println(vision_ok ? "READY" : "FAULT");

    display.setTextSize(1);
    display.setCursor(0, 24);
    display.println(vision_ok ? "Vision AI OK" : "Vision AI I2C");

    display.display();
}


HardwareSerial BrokerUART(1);

/* ================================
   TRANSPORT
   ================================ */
static constexpr uint32_t ACK_TIMEOUT_MS = 5000;
static constexpr uint8_t  MAX_ACK_TIMEOUT_RETRIES = 5;

/* ================================
   STATE
   ================================ */
static uint32_t frame_id = 0;
static bool     awaiting_ack = false;
static uint32_t last_send_ms = 0;

/* timeout retry tracking + transport pause */
static uint8_t ack_timeout_retries = 0;
static bool    transport_paused = false;

/* actuation debounce by consecutive detections */
static uint8_t actuation_streak_led1 = 0;
static uint8_t actuation_streak_led2 = 0;
static uint8_t actuation_streak_led3 = 0;
static bool actuation_armed_led1 = true;
static bool actuation_armed_led2 = true;
static bool actuation_armed_led3 = true;

/* Cached frame (for resend) */
static String cached_json;
static String cached_inf;
static String cached_image;
static size_t cached_image_len = 0;
static uint32_t cached_image_crc = 0;
static bool image_type_logged = false;

bool web_frame_available()
{
    return cached_image_len > 0;
}

const uint8_t *web_frame_ptr()
{
    return (const uint8_t *)cached_image.c_str();
}

size_t web_frame_len()
{
    return cached_image_len;
}

bool web_inf_available()
{
    return cached_inf.length() > 0;
}

const char *web_inf_ptr()
{
    return cached_inf.c_str();
}

size_t web_inf_len()
{
    return cached_inf.length();
}

/* UART RX line assembly (non-blocking) */
static String uart_line;

static void oled_show_ready_fault_if_idle()
{
    if (bootTestActive())
        return;

    if (stepper.active)
        return;

    uint32_t now = millis();
    uint32_t base = (last_inference_ms > 0) ? last_inference_ms : boot_end_ms;

    if ((int32_t)(now - (base + BOOT_STATUS_TIMEOUT_MS)) < 0)
        return;

    if (awaiting_ack)
        return;

    oled_show_ready_fault(vision_ai_ok);
}

/* ================================
   DIP SWITCH (Grove 6-DIP on I2C)
   ================================ */
static GroveMultiSwitch mswitch;
static const char* grove_5way_tactile_keys[] = {
    "KEY A",
    "KEY B",
    "KEY C",
    "KEY D",
    "KEY E",
};
static const char* grove_6pos_dip_switch_keys[] = {
    "POS 1",
    "POS 2",
    "POS 3",
    "POS 4",
    "POS 5",
    "POS 6",
};
static const char** key_names = nullptr;
static uint8_t mswitch_count = 0;
static uint16_t mswitch_pid = 0;
static uint8_t mswitch_last_mask = 0xFF;
static uint32_t mswitch_last_ms = 0;
static constexpr uint32_t MSWITCH_LOG_INTERVAL_MS = 1000;

static int deviceDetect()
{
    if (!mswitch.begin())
    {
        Serial.println("***** Grove Multi Switch probe failed *****");
        mswitch_count = 0;
        mswitch_pid = 0;
        key_names = nullptr;
        apply_default_stepper();
        return -1;
    }

    Serial.println("***** Grove Multi Switch probe OK *****");
    mswitch_pid = PID_VAL(mswitch.getDevID());
    if (mswitch_pid == PID_5_WAY_TACTILE_SWITCH)
    {
        Serial.println("Grove 5-Way Tactile Switch Inserted!");
        key_names = grove_5way_tactile_keys;
    }
    else if (mswitch_pid == PID_6_POS_DIP_SWITCH)
    {
        Serial.println("Grove 6-Position DIP Switch Inserted!");
        key_names = grove_6pos_dip_switch_keys;
    }
    else
    {
        Serial.println("Unknown Grove Multi Switch device");
        key_names = nullptr;
    }

    mswitch.setEventMode(true);

    mswitch_count = mswitch.getSwitchCount();
    Serial.print("A ");
    Serial.print(mswitch_count);
    Serial.print(" Button/Switch Device ");
    Serial.println(mswitch.getDevVer());
    return 0;
}

/* ================================
   UTIL
   ================================ */
void log_memory()
{
    Serial.printf(
        "heap_free=%u heap_min=%u psram=%s\n",
        ESP.getFreeHeap(),
        ESP.getMinFreeHeap(),
        psramFound() ? "YES" : "NO"
    );
}

static const char* target_to_label(uint8_t target)
{
    switch (target)
    {
        case 3: return "Vespa velutina (3)";
        case 2: return "Vespula spp. (2)";
        case 1: return "Vespa crabro (1)";
        case 0: return "Apis mellifera (0)";
        default: return "Unknown";
    }
}

/* ================================
   OLED DRAW
   ================================ */
static void oled_show(uint8_t target, uint8_t score)
{
    if (!OLED_OK) return;

    // Avoid excessive redraws (OLED + I2C) if nothing changes.
    // Still allow occasional refresh.
    uint32_t now = millis();
    bool changed = (target != oled_last_target) || (score != oled_last_score);
    bool stale = (now - oled_last_ms) > 1000;

    if (!changed && !stale) return;

    oled_last_target = target;
    oled_last_score  = score;
    oled_have_detection = true;
    oled_last_ms     = now;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // Big confidence caption (top)
    // Format: "86%"
    display.setTextSize(3);
    display.setCursor(0, 0);
    display.printf("%u%%", (unsigned)score);

    if (stepper.active)
    {
        // Right-aligned direction indicator next to percentage
        const char *sym = motorLabelForOled();
        display.setTextSize(2);
        int16_t x1, y1;
        uint16_t w, h;
        display.getTextBounds(sym, 0, 0, &x1, &y1, &w, &h);
        int16_t x = (int16_t)(OLED_W - w);
        int16_t y = 2;
        if (x < 0) x = 0;
        display.setCursor(x, y);
        display.print(sym);
    }

    // Species label below
    display.setTextSize(1);
    display.setCursor(0, 34);
    display.println(target_to_label(target));

    // Optional small status line (keeps main request intact, but helpful)
    display.setCursor(0, 52);
    if (stepperInCooldown())
    {
        display.print("CoolDown");
    }
    else if (stepper.active &&
             stepperInPause() &&
             stepper.wait_ms > 0)
    {
        uint32_t rem = oled_wait_remaining_seconds();
        display.print("Wait ");
        display.print(rem);
        display.print("s");
    }
    else
    {
        display.print("Trh ");
        display.print(CONFIDENCE_THRESHOLD);
        display.print("%");
        if (ENABLE_UART_TRANSPORT)
        {
            display.print(" ");
            display.print(transport_paused ? "UART PAUSE" : "UART OK");
        }
    }

    display.display();
}

static void oled_show_no_detection()
{
    if (!OLED_OK) return;

    uint32_t now = millis();
    bool stale = (now - oled_last_ms) > 1000;
    if (!stale) return;

    oled_last_target = 255;
    oled_last_score  = 255;
    oled_last_ms     = now;
    oled_have_detection = false;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("--");

    if (stepper.active)
    {
        // Right-aligned direction indicator
        const char *sym = motorLabelForOled();
        int16_t x1, y1;
        uint16_t w, h;
        display.getTextBounds(sym, 0, 0, &x1, &y1, &w, &h);
        int16_t x = (int16_t)(OLED_W - w);
        int16_t y = 0;
        if (x < 0) x = 0;
        display.setCursor(x, y);
        display.print(sym);
    }

    display.setTextSize(1);
    display.setCursor(0, 30);
    display.println("No detection");

    display.setCursor(0, 52);
    if (stepperInCooldown())
    {
        display.print("CoolDown");
    }
    else if (stepper.active &&
             stepperInPause() &&
             stepper.wait_ms > 0)
    {
        uint32_t rem = oled_wait_remaining_seconds();
        display.print("Wait ");
        display.print(rem);
        display.print("s");
    }
    else
    {
        display.print("thr ");
        display.print(CONFIDENCE_THRESHOLD);
        display.print("%");
        if (ENABLE_UART_TRANSPORT)
        {
            display.print(" ");
            display.print(transport_paused ? "UART PAUSE" : "UART OK");
        }
    }

    display.display();
}

/* ================================
   ACTUATOR TIMER CALLBACKS
   ================================ */
static void led1_off_cb(void *arg)
{
    (void)arg;
    digitalWrite(LED_PIN_1, LOW);
    led1_until = 0;
}
static void led2_off_cb(void *arg)
{
    (void)arg;
    digitalWrite(LED_PIN_2, LOW);
    led2_until = 0;
}
static void led3_off_cb(void *arg)
{
    (void)arg;
    digitalWrite(LED_PIN_3, LOW);
    led3_until = 0;
}

/* Trigger helper: immediate ON + scheduled OFF even if loop is busy */
static inline void trigger_actuator(int pin, esp_timer_handle_t tmr, uint32_t &until_ms)
{
    digitalWrite(pin, HIGH);
    uint32_t now = millis();
    until_ms = now + LED_ON_MS;

    if (tmr)
    {
        esp_timer_stop(tmr);
        esp_timer_start_once(tmr, (uint64_t)LED_ON_MS * 1000ULL);
    }
}

static void apply_dip_to_stepper(uint8_t dip_mask)
{
    // Mapping (LSB order = switch1..6):
    // bits 1:0 = direction, bits 3:2 = angle, bits 5:4 = wait
    uint8_t dir_idx = dip_mask & 0x03;
    uint8_t angle_idx = (dip_mask >> 2) & 0x03;
    uint8_t wait_idx = (dip_mask >> 4) & 0x03;

    stepper.direction_mode = DIP_DIR_MAP[dir_idx];
    stepper.wait_ms = DIP_WAIT_MS[wait_idx];

    uint16_t angle_deg = DIP_ANGLE_DEG[angle_idx];
    uint32_t steps = (stepper.steps_per_rev * (uint32_t)angle_deg) / 360U;
    if (steps == 0) steps = 1;
    stepper.steps_per_move = steps;
}

static const char* dirLabel(uint8_t mode)
{
    switch (mode)
    {
        case DIP_DIR_CCW_ONLY: return "RV";
        case DIP_DIR_CW_ONLY: return "FW";
        case DIP_DIR_CCW_THEN_CW: return "RV->FW";
        case DIP_DIR_CW_THEN_CCW: return "FW->RV";
        default: return "?";
    }
}

static bool motorDirIsRev()
{
    if (stepper.phase == STEPPER_FWD)
        return false;
    if (stepper.phase == STEPPER_REV)
        return true;

    // During pause, show the upcoming direction when two-direction mode is active
    if (stepper.phase == STEPPER_PAUSE_FWD)
        return (stepper.direction_mode == DIP_DIR_CW_THEN_CCW);
    if (stepper.phase == STEPPER_PAUSE_REV)
        return (stepper.direction_mode == DIP_DIR_CCW_THEN_CW);

    return stepper.last_dir_rev;
}

static bool stepperInPause()
{
    return (stepper.phase == STEPPER_PAUSE_FWD || stepper.phase == STEPPER_PAUSE_REV);
}

static const char* motorLabelForOled()
{
    if (stepperInPause())
        return "HOLD";
    return motorDirIsRev() ? "RV" : "FW";
}

static uint32_t oled_wait_remaining_seconds()
{
    uint32_t now = millis();
    uint32_t rem = 0;
    if ((int32_t)(stepper.pause_until_ms - now) > 0)
        rem = (stepper.pause_until_ms - now + 999) / 1000;
    if (rem > 0)
        rem -= 1;
    return rem;
}

static bool apply_dip_from_switch()
{
    if (mswitch_count == 0)
        return false;

    GroveMultiSwitch::ButtonEvent_t* evt = mswitch.getEvent();
    if (!evt)
        return false;

    uint8_t raw_mask = 0;
    for (uint8_t i = 0; i < mswitch_count; i++)
    {
        if (evt->button[i] & GroveMultiSwitch::BTN_EV_RAW_STATUS)
            raw_mask |= (1U << i);
    }

    uint8_t on_mask = (uint8_t)(~raw_mask) & ((1U << mswitch_count) - 1U);
    mswitch_last_mask = on_mask;
    mswitch_last_ms = millis();

    if (mswitch_count >= 6)
    {
        apply_dip_to_stepper((uint8_t)(on_mask & 0x3F));
        return true;
    }

    return false;
}

/* ================================
   SEND FRAME (CACHED)
   ================================ */
void send_cached_frame()
{
    if (!ENABLE_UART_TRANSPORT)
        return;

    if (transport_paused)
        return;

    BrokerUART.print("JSON ");
    BrokerUART.println(cached_json);

    BrokerUART.print("INF ");
    BrokerUART.println(cached_inf);

    BrokerUART.print("IMAGE ");
    BrokerUART.print(cached_image_len);
    BrokerUART.print(" ");
    BrokerUART.printf("%08lx\n", cached_image_crc);
    BrokerUART.print(cached_image);

    BrokerUART.println("END");

    last_send_ms = millis();
    awaiting_ack = true;

    Serial.printf(
        "📤 frame %lu sent (%u bytes)\n",
        frame_id,
        (unsigned)cached_image_len
    );
}

/* ================================
   PREPARE NEXT FRAME
   ================================ */
bool prepare_frame()
{
    vision_ai_ok = visionAiProbeI2C();
    if (!vision_ai_ok)
        return false;

    int rc = AI.invoke(1, false, false);
    if (rc != CMD_OK)
        return false;
    last_inference_ms = millis();

    // Read Grove Multi Switch just before inference analysis/boxes output
    {
        GroveMultiSwitch::ButtonEvent_t* evt = mswitch.getEvent();
        if (!evt)
        {
            // try re-probe if device went away or wasn't ready
            deviceDetect();
        }
        else
        {
            if (mswitch_count > 0)
            {
                uint8_t raw_mask = 0;
                for (uint8_t i = 0; i < mswitch_count; i++)
                {
                    if (evt->button[i] & GroveMultiSwitch::BTN_EV_RAW_STATUS)
                        raw_mask |= (1U << i);
                }

                // DIP ON = LOW, so invert raw for a more intuitive mask
                uint8_t on_mask = (uint8_t)(~raw_mask) & ((1U << mswitch_count) - 1U);

                bool changed = (on_mask != mswitch_last_mask);
                uint32_t now = millis();
                bool stale = (now - mswitch_last_ms) > MSWITCH_LOG_INTERVAL_MS;

                if (changed || stale || (evt->event & GroveMultiSwitch::BTN_EV_HAS_EVENT))
                {
                    mswitch_last_mask = on_mask;
                    mswitch_last_ms = now;

                    char bits_lsb[7] = {0};
                    for (uint8_t i = 0; i < 6; i++)
                    {
                        bits_lsb[i] = (on_mask & (1U << i)) ? '1' : '0';      // switch1..6
                    }

                    // LSB shows switches 1..6 (left to right)
                    Serial.printf("🎛️ DIP mask: 0x%02X S1..S6:%s\n", on_mask, bits_lsb);

                    // Apply 6-bit DIP to stepper behavior (bits 0..5)
                    if (mswitch_count >= 6)
                    {
                        apply_dip_to_stepper((uint8_t)(on_mask & 0x3F));

                        uint8_t dir_idx = (on_mask & 0x03);
                        uint8_t angle_idx = (on_mask >> 2) & 0x03;
                        uint8_t wait_idx = (on_mask >> 4) & 0x03;
                        Serial.printf(
                            "🧭 DIR=%s ANG=%u WAIT=%lu\n",
                            dirLabel(DIP_DIR_MAP[dir_idx]),
                            (unsigned)DIP_ANGLE_DEG[angle_idx],
                            (unsigned long)DIP_WAIT_MS[wait_idx]
                        );
                    }
                }
            }
        }
    }

    Serial.println("🧠 RAW INFERENCE RESULT");
    Serial.printf("boxes: %u\n", (unsigned)AI.boxes().size());

    // Pick "best" box for OLED (highest score)
    uint8_t best_target = 0;
    uint8_t best_score  = 0;
    bool    have_best   = false;

    bool class1_detected = false;
    bool class2_detected = false;
    bool class3_detected = false;
    for (size_t i = 0; i < AI.boxes().size(); i++)
    {
        auto &b = AI.boxes()[i];
        Serial.printf(
            "  [%u] target=%u score=%u x=%u y=%u w=%u h=%u\n",
            (unsigned)i,
            b.target,
            b.score,
            b.x,
            b.y,
            b.w,
            b.h
        );

        // Update "best" for OLED
        if (!have_best || b.score > best_score)
        {
            have_best = true;
            best_score = b.score;
            best_target = b.target;
        }

        // Actuation (thresholded)
        if (b.target == CLASS_LED1 && b.score >= CONFIDENCE_THRESHOLD)
        {
            class1_detected = true;
        }
        else if (b.target == CLASS_LED2 && b.score >= CONFIDENCE_THRESHOLD)
        {
            class2_detected = true;
        }
        else if (b.target == CLASS_LED3 && b.score >= CONFIDENCE_THRESHOLD)
        {
            class3_detected = true;
        }
    }

    // Actuation gate: require consecutive detections per class
    if (!class1_detected)
    {
        actuation_streak_led1 = 0;
        actuation_armed_led1 = true;
    }
    else if (actuation_armed_led1)
    {
        if (actuation_streak_led1 < 255)
            actuation_streak_led1++;
        if (actuation_streak_led1 >= ACTUATION_REQUIRED_OCCURRENCES)
        {
            trigger_actuator(LED_PIN_1, led1_timer, led1_until);
            trigger_motor_cycle(false);
            actuation_armed_led1 = false; // re-arm after loss of detection
        }
    }

    if (!class2_detected)
    {
        actuation_streak_led2 = 0;
        actuation_armed_led2 = true;
    }
    else if (actuation_armed_led2)
    {
        if (actuation_streak_led2 < 255)
            actuation_streak_led2++;
        if (actuation_streak_led2 >= ACTUATION_REQUIRED_OCCURRENCES)
        {
            trigger_actuator(LED_PIN_2, led2_timer, led2_until);
            actuation_armed_led2 = false;
        }
    }

    if (!class3_detected)
    {
        actuation_streak_led3 = 0;
        actuation_armed_led3 = true;
    }
    else if (actuation_armed_led3)
    {
        if (actuation_streak_led3 < 255)
            actuation_streak_led3++;
        if (actuation_streak_led3 >= ACTUATION_REQUIRED_OCCURRENCES)
        {
            trigger_actuator(LED_PIN_3, led3_timer, led3_until);
            actuation_armed_led3 = false;
        }
    }

    // OLED update
    if (have_best)
        oled_show(best_target, best_score);
    else
        oled_show_no_detection();

    frame_id++;

    cached_inf = "";
    cached_inf += "{\"frame\":";
    cached_inf += frame_id;
    cached_inf += ",\"perf\":{";
    cached_inf += "\"preprocess\":";
    cached_inf += AI.perf().prepocess;
    cached_inf += ",\"inference\":";
    cached_inf += AI.perf().inference;
    cached_inf += ",\"postprocess\":";
    cached_inf += AI.perf().postprocess;
    cached_inf += "},\"boxes\":[";

    for (size_t i = 0; i < AI.boxes().size(); i++)
    {
        auto &b = AI.boxes()[i];
        if (i) cached_inf += ",";
        cached_inf += "{\"target\":";
        cached_inf += b.target;
        cached_inf += ",\"score\":";
        cached_inf += b.score;
        cached_inf += ",\"x\":";
        cached_inf += b.x;
        cached_inf += ",\"y\":";
        cached_inf += b.y;
        cached_inf += ",\"w\":";
        cached_inf += b.w;
        cached_inf += ",\"h\":";
        cached_inf += b.h;
        cached_inf += "}";
    }
    cached_inf += "]}";

    cached_json = cached_inf;

    bool want_image = (WEB_MODE != 0) || (ENABLE_UART_TRANSPORT && !transport_paused);

    if (!want_image)
    {
        cached_image = "";
        cached_image_len = 0;
        cached_image_crc = 0;
        return true;
    }

    cached_image = AI.last_image();
    cached_image_len = cached_image.length();

    cached_image_crc = esp_crc32_le(
        0,
        (const uint8_t*)cached_image.c_str(),
        cached_image_len
    );

    Serial.printf(
        "🧠 prepared frame %lu (img=%u, crc=%08lx)\n",
        frame_id,
        (unsigned)cached_image_len,
        cached_image_crc
    );

    if (!image_type_logged && cached_image_len >= 4)
    {
        const uint8_t *p = (const uint8_t *)cached_image.c_str();
        bool is_jpeg = (cached_image_len >= 4 &&
                        p[0] == 0xFF && p[1] == 0xD8 &&
                        p[cached_image_len - 2] == 0xFF && p[cached_image_len - 1] == 0xD9);
        bool is_png = (cached_image_len >= 8 &&
                       p[0] == 0x89 && p[1] == 0x50 && p[2] == 0x4E && p[3] == 0x47 &&
                       p[4] == 0x0D && p[5] == 0x0A && p[6] == 0x1A && p[7] == 0x0A);
        bool is_bmp = (cached_image_len >= 2 && p[0] == 'B' && p[1] == 'M');

        Serial.print("IMG magic: ");
        for (size_t i = 0; i < 16 && i < cached_image_len; i++)
        {
            Serial.printf("%02X ", p[i]);
        }
        Serial.println();

        if (is_jpeg) Serial.println("IMG type: JPEG");
        else if (is_png) Serial.println("IMG type: PNG");
        else if (is_bmp) Serial.println("IMG type: BMP");
        else Serial.println("IMG type: UNKNOWN");

        image_type_logged = true;
    }

    return true;
}

/* ================================
   UART RX LINE PROCESSING
   ================================ */
static void process_uart_line(const String &line)
{
    if (line.startsWith("ACK "))
    {
        uint32_t ack_id = line.substring(4).toInt();

        // Any ACK can be treated as "link is alive again" if we were paused
        if (transport_paused)
        {
            transport_paused = false;
            ack_timeout_retries = 0;
            awaiting_ack = false;
            Serial.printf("🔓 transport resumed on ACK %lu\n", ack_id);
        }

        if (ack_id == frame_id)
        {
            awaiting_ack = false;
            ack_timeout_retries = 0;
            Serial.printf("✅ ACK %lu\n", ack_id);
        }
    }
    else if (line.startsWith("NACK "))
    {
        uint32_t nack_id = line.substring(5).toInt();
        if (nack_id == frame_id)
        {
            Serial.printf("🔁 NACK %lu → resend\n", nack_id);
            send_cached_frame();
        }
    }
}

/* Non-blocking UART poll: assemble lines char-by-char */
static void poll_uart_nonblocking()
{
    while (BrokerUART.available())
    {
        char c = (char)BrokerUART.read();

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            uart_line.trim();
            if (uart_line.length() > 0)
            {
                process_uart_line(uart_line);
            }
            uart_line = "";
        }
        else
        {
            // prevent runaway memory usage if peer sends junk without newlines
            if (uart_line.length() < 200)
                uart_line += c;
        }
    }
}

/* ================================
   SETUP
   ================================ */
void setup()
{
    pinMode(LED_PIN_1, OUTPUT);
    pinMode(LED_PIN_2, OUTPUT);
    pinMode(LED_PIN_3, OUTPUT);

    // Create timers (one-shot)
    {
        esp_timer_create_args_t a1 = {};
        a1.callback = &led1_off_cb;
        a1.name = "led1_off";
        esp_timer_create(&a1, &led1_timer);

        esp_timer_create_args_t a2 = {};
        a2.callback = &led2_off_cb;
        a2.name = "led2_off";
        esp_timer_create(&a2, &led2_timer);

        esp_timer_create_args_t a3 = {};
        a3.callback = &led3_off_cb;
        a3.name = "led3_off";
        esp_timer_create(&a3, &led3_timer);
    }

    Serial.begin(115200);
    delay(2000);

    Serial.println("=======================================");
    Serial.println(" XIAO ESP32-S3 | SSCMA UART BROKER ");
    Serial.println("=======================================");

    web_init();

    if (ENABLE_UART_TRANSPORT)
    {
        BrokerUART.begin(
            UART_BAUD,
            SERIAL_8N1,
            UART_RX_PIN,
            UART_TX_PIN
        );
    }

    Wire.begin();
    Wire.setClock(400000);

    deviceDetect();
    apply_dip_from_switch();

    stepperInit();
    if (BOOT_MOTOR_CYCLE)
    {
        boot_motor_pending = true;
        boot_motor_deadline_ms = millis() + 500;
    }

    // OLED init (do NOT hard-fail if missing)
    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
    {
        OLED_OK = true;
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("OLED OK");
        display.print("thr ");
        display.print(CONFIDENCE_THRESHOLD);
        display.println("%");
        display.display();
        Serial.println("✅ OLED initialized");
    }
    else
    {
        OLED_OK = false;
        Serial.println("⚠️ OLED init failed (continuing without OLED)");
    }

    vision_ai_ok = visionAiProbeI2C();
    if (!vision_ai_ok || !AI.begin(&Wire))
    {
        Serial.println("❌ SSCMA init failed");
        vision_ai_ok = false;
    }
    else
    {
        vision_ai_ok = true;
        Serial.println("✅ SSCMA initialized");
        log_memory();
    }

    // Power-on blink (still works, now via trigger helper)
    trigger_actuator(LED_PIN_1, led1_timer, led1_until);
    trigger_actuator(LED_PIN_2, led2_timer, led2_until);
    trigger_actuator(LED_PIN_3, led3_timer, led3_until);

    boot_test_started = true;
    boot_end_ms = millis();
}

/* ================================
   LOOP
   ================================ */
void loop()
{
    web_loop();
    stepperUpdate();

    if (boot_motor_pending)
    {
        bool applied = apply_dip_from_switch();
        if (applied || (int32_t)(millis() - boot_motor_deadline_ms) >= 0)
        {
            trigger_motor_cycle(true);
            boot_motor_pending = false;
        }
    }

    // If stepper is running, pause inference to avoid I2C contention
    if (stepper.active)
    {
        if (bootTestActive())
        {
            oled_show_boot();
            return;
        }

        // Keep OLED updated during motor motion/pause
        if (oled_have_detection)
            oled_show(oled_last_target, oled_last_score);
        else
            oled_show_no_detection();

        // Still service UART RX/ACK to keep link alive
        if (ENABLE_UART_TRANSPORT)
        {
            poll_uart_nonblocking();

            if (awaiting_ack && (millis() - last_send_ms > ACK_TIMEOUT_MS))
            {
                ack_timeout_retries++;
                if (ack_timeout_retries >= MAX_ACK_TIMEOUT_RETRIES)
                {
                    transport_paused = true;
                    awaiting_ack = false;
                }
                else
                {
                    send_cached_frame();
                }
            }
        }
        return;
    }

    if (bootTestActive())
    {
        oled_show_boot();
    }
    else
    {
        oled_show_ready_fault_if_idle();
    }

    // UART RX (non-blocking)
    if (ENABLE_UART_TRANSPORT)
    {
        poll_uart_nonblocking();

        // ACK timeout / resend / pause logic
        if (awaiting_ack && (millis() - last_send_ms > ACK_TIMEOUT_MS))
        {
            ack_timeout_retries++;

            if (ack_timeout_retries >= MAX_ACK_TIMEOUT_RETRIES)
            {
                Serial.printf(
                    "⏱ ACK timeout x%u for frame %lu → STOP SENDING, keep inference running (skip images)\n",
                    ack_timeout_retries,
                    frame_id
                );

                transport_paused = true;
                awaiting_ack = false;
                return;
            }

            Serial.printf(
                "⏱ ACK timeout for frame %lu (retry %u/%u) → resend\n",
                frame_id,
                ack_timeout_retries,
                MAX_ACK_TIMEOUT_RETRIES
            );
            send_cached_frame();
            return;
        }
    }

    // Prepare new frame only when we are not waiting on ACK
    if (!awaiting_ack)
    {
        if (prepare_frame())
        {
            if (ENABLE_UART_TRANSPORT)
            {
                // send_cached_frame() will no-op if transport_paused==true
                send_cached_frame();
            }
        }
    }

    stepperUpdate();
}
