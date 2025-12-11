/**
 * RC Vliegtuig - ESP32 Onboard Controller
 * Modern C++ implementatie met RAII en data-oriented design
 */

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <array>
#include <optional>

// ============================================================================
// Configuration & Constants
// ============================================================================

namespace Config {
constexpr uint8_t ESC_PIN = 13;
constexpr uint8_t AILERON_PIN = 12;
constexpr uint8_t ELEVATOR_PIN = 14;
constexpr uint8_t RUDDER_PIN = 27;
constexpr uint8_t BATTERY_PIN = 34;

constexpr uint32_t FAILSAFE_TIMEOUT_MS = 1000;
constexpr uint32_t BATTERY_CHECK_INTERVAL_MS = 1000;
constexpr uint32_t STATUS_UPDATE_INTERVAL_MS = 2000;

constexpr float LOW_VOLTAGE_THRESHOLD = 7.0f;
constexpr float CRITICAL_VOLTAGE_THRESHOLD = 6.5f;
constexpr float VOLTAGE_DIVIDER_RATIO = 3.0f;
constexpr float ADC_VOLTAGE_REF = 3.3f;
constexpr float ADC_MAX_VALUE = 4095.0f;

constexpr int SERVO_MIN = 1000;
constexpr int SERVO_MAX = 2000;
constexpr int SERVO_NEUTRAL = 1500;
} // namespace Config

// ============================================================================
// Data Structures (Data-Oriented Design)
// ============================================================================

struct ControlData {
    int throttle{0};  // 0-180
    int aileron{90};  // 0-180, 90 = neutral
    int elevator{90}; // 0-180, 90 = neutral
    int rudder{90};   // 0-180, 90 = neutral
};

struct TrimData {
    int aileron{0};  // -30 to +30
    int elevator{0}; // -30 to +30
    int rudder{0};   // -30 to +30
};

struct BatteryData {
    float voltage{0.0f};
    bool is_low{false};
    bool is_critical{false};
};

struct SystemState {
    bool esc_armed{false};
    uint32_t last_command_time{0};
    BatteryData battery;
};

// ============================================================================
// RAII Wrappers
// ============================================================================

class ServoController {
    Servo servo_;
    uint8_t pin_;

  public:
    explicit ServoController(uint8_t pin) : pin_(pin) {
        servo_.attach(pin_, Config::SERVO_MIN, Config::SERVO_MAX);
    }

    ~ServoController() {
        servo_.detach();
    }

    ServoController(const ServoController &) = delete;
    ServoController &operator=(const ServoController &) = delete;

    void write(int value) {
        servo_.write(constrain(value, 0, 180));
    }
};

class PreferencesStore {
    Preferences prefs_;

  public:
    PreferencesStore() {
        prefs_.begin("rc-plane", false);
    }

    ~PreferencesStore() {
        prefs_.end();
    }

    PreferencesStore(const PreferencesStore &) = delete;
    PreferencesStore &operator=(const PreferencesStore &) = delete;

    TrimData load_trims() {
        TrimData trims;
        trims.aileron = prefs_.getInt("trim_a", 0);
        trims.elevator = prefs_.getInt("trim_e", 0);
        trims.rudder = prefs_.getInt("trim_r", 0);
        return trims;
    }

    void save_trims(const TrimData &trims) {
        prefs_.putInt("trim_a", trims.aileron);
        prefs_.putInt("trim_e", trims.elevator);
        prefs_.putInt("trim_r", trims.rudder);
    }
};

// ============================================================================
// Command Parser
// ============================================================================

class CommandParser {
  public:
    static std::optional<ControlData> parse_control(const String &cmd) {
        if (!validate_checksum(cmd))
            return std::nullopt;

        ControlData data;
        data.throttle = extract_value(cmd, "T:");
        data.aileron = extract_value(cmd, "A:");
        data.elevator = extract_value(cmd, "E:");
        data.rudder = extract_value(cmd, "R:");

        return data;
    }

    static std::optional<TrimData> parse_trim(const String &cmd) {
        TrimData data;
        data.aileron = extract_value(cmd, "A:");
        data.elevator = extract_value(cmd, "E:");
        data.rudder = extract_value(cmd, "R:");
        return data;
    }

  private:
    static bool validate_checksum(const String &cmd) {
        int chk_idx = cmd.indexOf("CHK:");
        if (chk_idx == -1)
            return false;

        String data = cmd.substring(0, chk_idx);
        String checksum_str = cmd.substring(chk_idx + 4);

        int calculated = 0;
        for (char c : data)
            calculated += c;

        return calculated == checksum_str.toInt();
    }

    static int extract_value(const String &cmd, const char *prefix) {
        int idx = cmd.indexOf(prefix);
        if (idx == -1)
            return 90; // neutral default

        int comma = cmd.indexOf(',', idx);
        int chk = cmd.indexOf("CHK:");
        int end = (comma != -1 && comma < chk) ? comma : chk;

        return cmd.substring(idx + 2, end).toInt();
    }
};

// ============================================================================
// Battery Monitor
// ============================================================================

class BatteryMonitor {
  public:
    BatteryData read() {
        int raw = analogRead(Config::BATTERY_PIN);
        float voltage = (raw / Config::ADC_MAX_VALUE) *
                        Config::ADC_VOLTAGE_REF * Config::VOLTAGE_DIVIDER_RATIO;

        BatteryData data;
        data.voltage = voltage;
        data.is_low = voltage < Config::LOW_VOLTAGE_THRESHOLD;
        data.is_critical = voltage < Config::CRITICAL_VOLTAGE_THRESHOLD;

        return data;
    }
};

// ============================================================================
// Main Controller
// ============================================================================

class PlaneController {
    ServoController esc_{Config::ESC_PIN};
    ServoController aileron_{Config::AILERON_PIN};
    ServoController elevator_{Config::ELEVATOR_PIN};
    ServoController rudder_{Config::RUDDER_PIN};

    BluetoothSerial bt_;
    PreferencesStore prefs_;
    BatteryMonitor battery_monitor_;

    ControlData controls_;
    TrimData trims_;
    SystemState state_;

    uint32_t last_battery_check_{0};
    uint32_t last_status_update_{0};

  public:
    PlaneController() {
        bt_.begin("RC-Plane");
        trims_ = prefs_.load_trims();
        init_esc();
        apply_controls();
        Serial.println("Plane controller started");
    }

    void loop() {
        handle_commands();
        check_failsafe();

        if (millis() - last_battery_check_ >
            Config::BATTERY_CHECK_INTERVAL_MS) {
            update_battery();
            last_battery_check_ = millis();
        }

        if (millis() - last_status_update_ >
            Config::STATUS_UPDATE_INTERVAL_MS) {
            send_status();
            last_status_update_ = millis();
        }
    }

  private:
    void init_esc() {
        esc_.write(0);
        delay(1000);
        Serial.println("ESC initialized");
    }

    void handle_commands() {
        if (!bt_.available())
            return;

        String cmd = bt_.readStringUntil('\n');
        state_.last_command_time = millis();

        if (cmd.indexOf("ARM") != -1) {
            arm();
        } else if (cmd.indexOf("DISARM") != -1) {
            disarm();
        } else if (cmd.indexOf("TRIM:") != -1) {
            if (auto trim = CommandParser::parse_trim(cmd)) {
                trims_ = *trim;
                prefs_.save_trims(trims_);
                apply_controls();
            }
        } else if (auto ctrl = CommandParser::parse_control(cmd)) {
            controls_ = *ctrl;
            apply_controls();
        }
    }

    void apply_controls() {
        int throttle = state_.esc_armed ? controls_.throttle : 0;
        int aileron = constrain(controls_.aileron + trims_.aileron, 0, 180);
        int elevator = constrain(controls_.elevator + trims_.elevator, 0, 180);
        int rudder = constrain(controls_.rudder + trims_.rudder, 0, 180);

        esc_.write(throttle);
        aileron_.write(aileron);
        elevator_.write(elevator);
        rudder_.write(rudder);
    }

    void check_failsafe() {
        if (millis() - state_.last_command_time > Config::FAILSAFE_TIMEOUT_MS) {
            activate_failsafe();
        }
    }

    void activate_failsafe() {
        Serial.println("FAILSAFE - Connection lost");
        controls_ = ControlData{}; // Reset to neutral
        state_.esc_armed = false;
        apply_controls();
    }

    void arm() {
        if (!state_.esc_armed) {
            state_.esc_armed = true;
            Serial.println("ESC ARMED");
            bt_.println("STATUS:ARMED");
        }
    }

    void disarm() {
        if (state_.esc_armed) {
            state_.esc_armed = false;
            controls_.throttle = 0;
            apply_controls();
            Serial.println("ESC DISARMED");
            bt_.println("STATUS:DISARMED");
        }
    }

    void update_battery() {
        state_.battery = battery_monitor_.read();

        if (state_.battery.is_low) {
            Serial.printf("WARNING: Low battery: %.1fV\n",
                          state_.battery.voltage);
            bt_.printf("ALERT:LOWBAT:%.1f\n", state_.battery.voltage);
        }

        if (state_.battery.is_critical) {
            disarm();
        }
    }

    void send_status() {
        bt_.printf("STATUS:%s,BAT:%.1f,TRIM:A:%d,E:%d,R:%d\n",
                   state_.esc_armed ? "ARMED" : "DISARMED",
                   state_.battery.voltage, trims_.aileron, trims_.elevator,
                   trims_.rudder);
    }
};

// ============================================================================
// Entry Point
// ============================================================================

PlaneController *controller = nullptr;

void setup() {
    Serial.begin(115200);
    controller = new PlaneController();
}

void loop() {
    controller->loop();
}
