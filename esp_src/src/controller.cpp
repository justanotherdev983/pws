/**
 * RC Vliegtuig - ESP32 Remote Controller
 * Modern C++ implementatie met RAII en data-oriented design
 *
 * Controls:
 * - Left Joystick: X-axis = Aileron, Y-axis = Elevator
 * - Right Joystick: X-axis = Rudder, Y-axis = Throttle
 * - Button: ARM/DISARM toggle
 */

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <array>
#include <optional>

// ============================================================================
// Configuration & Constants
// ============================================================================

namespace Config {
// Left joystick (Aileron + Elevator)
constexpr uint8_t LEFT_X_PIN = 34; // Aileron
constexpr uint8_t LEFT_Y_PIN = 35; // Elevator

// Right joystick (Rudder + Throttle)
constexpr uint8_t RIGHT_X_PIN = 32; // Rudder
constexpr uint8_t RIGHT_Y_PIN = 33; // Throttle

// Button
constexpr uint8_t ARM_BUTTON_PIN = 15;

// LED
constexpr uint8_t STATUS_LED_PIN = 2;

constexpr uint32_t TRANSMIT_INTERVAL_MS = 50; // 20Hz
constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;
constexpr uint32_t CONNECTION_TIMEOUT_MS = 3000;

constexpr int ADC_MAX = 4095;
constexpr int DEADZONE = 100; // ADC units
constexpr float LOW_VOLTAGE_THRESHOLD = 7.0f;
} // namespace Config

// ============================================================================
// Data Structures
// ============================================================================

struct JoystickRaw {
    int left_x{0};
    int left_y{0};
    int right_x{0};
    int right_y{0};
};

struct ControlValues {
    int throttle{0};  // 0-180
    int aileron{90};  // 0-180, 90 = neutral
    int elevator{90}; // 0-180, 90 = neutral
    int rudder{90};   // 0-180, 90 = neutral
};

struct CalibrationData {
    int left_x_center{2048};
    int left_y_center{2048};
    int right_x_center{2048};
    int throttle_min{0};
    int throttle_max{4095};
};

struct PlaneStatus {
    bool armed{false};
    float battery_voltage{0.0f};
    bool connection_active{false};
    uint32_t last_received_time{0};
};

// ============================================================================
// RAII Wrappers
// ============================================================================

class PreferencesStore {
    Preferences prefs_;

  public:
    PreferencesStore() {
        prefs_.begin("rc-controller", false);
    }

    ~PreferencesStore() {
        prefs_.end();
    }

    PreferencesStore(const PreferencesStore &) = delete;
    PreferencesStore &operator=(const PreferencesStore &) = delete;

    CalibrationData load_calibration() {
        CalibrationData cal;
        cal.left_x_center = prefs_.getInt("cal_lx", 2048);
        cal.left_y_center = prefs_.getInt("cal_ly", 2048);
        cal.right_x_center = prefs_.getInt("cal_rx", 2048);
        cal.throttle_min = prefs_.getInt("cal_tmin", 0);
        cal.throttle_max = prefs_.getInt("cal_tmax", 4095);
        return cal;
    }

    void save_calibration(const CalibrationData &cal) {
        prefs_.putInt("cal_lx", cal.left_x_center);
        prefs_.putInt("cal_ly", cal.left_y_center);
        prefs_.putInt("cal_rx", cal.right_x_center);
        prefs_.putInt("cal_tmin", cal.throttle_min);
        prefs_.putInt("cal_tmax", cal.throttle_max);
    }
};

class Button {
    uint8_t pin_;
    bool last_state_{HIGH};
    uint32_t last_change_time_{0};

  public:
    explicit Button(uint8_t pin) : pin_(pin) {
        pinMode(pin_, INPUT_PULLUP);
    }

    bool pressed() {
        bool current = digitalRead(pin_);

        if (current != last_state_ &&
            millis() - last_change_time_ > Config::BUTTON_DEBOUNCE_MS) {
            last_change_time_ = millis();
            last_state_ = current;

            if (current == LOW) { // Button pressed (active low)
                return true;
            }
        }

        return false;
    }
};

// ============================================================================
// Joystick Reader
// ============================================================================

class JoystickReader {
    CalibrationData calibration_;

  public:
    explicit JoystickReader(const CalibrationData &cal) : calibration_(cal) {}

    JoystickRaw read_raw() const {
        JoystickRaw raw;
        raw.left_x = analogRead(Config::LEFT_X_PIN);
        raw.left_y = analogRead(Config::LEFT_Y_PIN);
        raw.right_x = analogRead(Config::RIGHT_X_PIN);
        raw.right_y = analogRead(Config::RIGHT_Y_PIN);
        return raw;
    }

    ControlValues process(const JoystickRaw &raw) const {
        ControlValues vals;

        // Left stick X-axis -> Aileron (centered)
        int aileron_offset = raw.left_x - calibration_.left_x_center;
        if (abs(aileron_offset) < Config::DEADZONE)
            aileron_offset = 0;
        vals.aileron = 90 + map(aileron_offset, -2048, 2048, -45, 45);
        vals.aileron = constrain(vals.aileron, 45, 135);

        // Left stick Y-axis -> Elevator (centered, inverted)
        int elevator_offset = -(raw.left_y - calibration_.left_y_center);
        if (abs(elevator_offset) < Config::DEADZONE)
            elevator_offset = 0;
        vals.elevator = 90 + map(elevator_offset, -2048, 2048, -45, 45);
        vals.elevator = constrain(vals.elevator, 45, 135);

        // Right stick X-axis -> Rudder (centered)
        int rudder_offset = raw.right_x - calibration_.right_x_center;
        if (abs(rudder_offset) < Config::DEADZONE)
            rudder_offset = 0;
        vals.rudder = 90 + map(rudder_offset, -2048, 2048, -45, 45);
        vals.rudder = constrain(vals.rudder, 45, 135);

        // Right stick Y-axis -> Throttle (not centered, inverted)
        vals.throttle =
            map(constrain(raw.right_y, calibration_.throttle_min,
                          calibration_.throttle_max),
                calibration_.throttle_min, calibration_.throttle_max,
                180, // Stick down = max throttle
                0    // Stick up = min throttle
            );
        vals.throttle = constrain(vals.throttle, 0, 180);

        return vals;
    }
};

// ============================================================================
// Status Parser
// ============================================================================

class StatusParser {
  public:
    static PlaneStatus parse(const String &msg, PlaneStatus current) {
        PlaneStatus status = current;
        status.last_received_time = millis();
        status.connection_active = true;

        if (msg.indexOf("STATUS:") != -1) {
            status.armed = msg.indexOf("ARMED") != -1;

            int bat_idx = msg.indexOf("BAT:");
            if (bat_idx != -1) {
                int comma = msg.indexOf(',', bat_idx);
                status.battery_voltage =
                    msg.substring(bat_idx + 4, comma).toFloat();
            }
        }

        if (msg.indexOf("ALERT:LOWBAT:") != -1) {
            int val_start = msg.indexOf("LOWBAT:") + 7;
            status.battery_voltage = msg.substring(val_start).toFloat();
            Serial.printf("WARNING: Low battery: %.1fV\n",
                          status.battery_voltage);
        }

        return status;
    }
};

// ============================================================================
// Command Builder
// ============================================================================

class CommandBuilder {
  public:
    static String build_control(const ControlValues &vals) {
        String cmd = String("T:") + vals.throttle + ",A:" + vals.aileron +
                     ",E:" + vals.elevator + ",R:" + vals.rudder;

        int checksum = 0;
        for (char c : cmd)
            checksum += c;
        cmd += ",CHK:" + String(checksum);

        return cmd;
    }

    static String build_arm() {
        return "ARM";
    }
    static String build_disarm() {
        return "DISARM";
    }
};

// ============================================================================
// LED Controller
// ============================================================================

class LEDController {
    uint8_t pin_;

  public:
    explicit LEDController(uint8_t pin) : pin_(pin) {
        pinMode(pin_, OUTPUT);
    }

    void update(const PlaneStatus &status) {
        if (!status.connection_active) {
            // No connection - slow blink
            digitalWrite(pin_, (millis() / 500) % 2);
        } else if (status.battery_voltage > 0.0f &&
                   status.battery_voltage < Config::LOW_VOLTAGE_THRESHOLD) {
            // Low battery - fast blink
            digitalWrite(pin_, (millis() / 150) % 2);
        } else if (status.armed) {
            // Armed - solid on
            digitalWrite(pin_, HIGH);
        } else {
            // Disarmed - off
            digitalWrite(pin_, LOW);
        }
    }
};

// ============================================================================
// Main Controller
// ============================================================================

class RemoteController {
    BluetoothSerial bt_;
    PreferencesStore prefs_;
    Button arm_button_{Config::ARM_BUTTON_PIN};
    LEDController led_{Config::STATUS_LED_PIN};

    CalibrationData calibration_;
    JoystickReader joystick_;
    PlaneStatus plane_status_;

    uint32_t last_transmit_{0};
    uint32_t last_debug_{0};

  public:
    RemoteController()
        : calibration_(prefs_.load_calibration()), joystick_(calibration_) {
        bt_.begin("RC-Controller");
        Serial.println("Remote controller started");
    }

    void loop() {
        handle_button();
        transmit_controls();
        receive_status();
        check_connection();
        led_.update(plane_status_);
    }

  private:
    void handle_button() {
        if (arm_button_.pressed()) {
            String cmd = plane_status_.armed ? CommandBuilder::build_disarm()
                                             : CommandBuilder::build_arm();
            bt_.println(cmd);
            Serial.println(cmd);
        }
    }

    void transmit_controls() {
        if (millis() - last_transmit_ < Config::TRANSMIT_INTERVAL_MS)
            return;

        JoystickRaw raw = joystick_.read_raw();
        ControlValues vals = joystick_.process(raw);
        String cmd = CommandBuilder::build_control(vals);

        bt_.println(cmd);
        last_transmit_ = millis();

        // Debug output once per second
        if (millis() - last_debug_ > 1000) {
            Serial.printf("T:%d A:%d E:%d R:%d | Armed:%d Batt:%.1fV\n",
                          vals.throttle, vals.aileron, vals.elevator,
                          vals.rudder, plane_status_.armed,
                          plane_status_.battery_voltage);
            last_debug_ = millis();
        }
    }

    void receive_status() {
        while (bt_.available()) {
            String msg = bt_.readStringUntil('\n');
            plane_status_ = StatusParser::parse(msg, plane_status_);
        }
    }

    void check_connection() {
        if (plane_status_.connection_active &&
            millis() - plane_status_.last_received_time >
                Config::CONNECTION_TIMEOUT_MS) {
            plane_status_.connection_active = false;
            Serial.println("Connection lost");
        }
    }
};

// ============================================================================
// Entry Point
// ============================================================================

RemoteController *controller = nullptr;

void setup() {
    Serial.begin(115200);
    delay(100);
    controller = new RemoteController();
}

void loop() {
    controller->loop();
}
