# RC Vliegtuig - Technisch Ontwerp (ESP32)

## Systeemarchitectuur

### Overzicht
Het RC vliegtuigproject bestaat uit twee hoofdcomponenten:
1. **Vliegtuig**: Met ESP32 microcontroller die de motor (via ESC) en servo's aanstuurt
2. **Controller**: Met ESP32 microcontroller die gebruikersinput verwerkt en verzendt

De twee componenten communiceren draadloos via **ESP-NOW** (ingebouwd in ESP32) voor lage latency en betrouwbare communicatie.

### Communicatiediagram
```
+----------------------+                    +----------------------+
|     CONTROLLER       |                    |      VLIEGTUIG       |
|                      |                    |                      |
|  +--------------+    |     ESP-NOW        |  +--------------+    |
|  | ESP32        |<---+------------------->+--| ESP32        |    |
|  | - Joysticks  |    |   2.4GHz WiFi      |  | - ESC/Motor  |    |
|  | - Knoppen    |    |   <1ms latency     |  | - Servo's    |    |
|  | - Display    |    |                    |  | - 3S LiPo    |    |
|  +--------------+    |                    |  | - Voltage Mon|    |
|  9V/LiPo voeding     |                    |  +--------------+    |
+----------------------+                    +----------------------+
```

## Hardwareontwerp

### Vliegtuig

#### Voeding en Motor
- **Batterij**: 3S LiPo 11.1V 2200mAh 25C (XT60 connector)
- **Motor**: 2312 980KV borstelloze outrunner motor
- **ESC**: 30A ESC (3S compatibel) met 5V/2A BEC
- **Propeller**: 9x4.5 of 10x4.5 inch (afhankelijk van gewicht)

#### Besturing
- **Microcontroller**: ESP32 DevKit (38 pin)
- **Servos**: 
  - 2x 9g micro servo (ailerons of één voor elevator)
  - 1x 9g micro servo (elevator of rudder)
  - 1x 9g micro servo (rudder - optioneel)
- **Voeding ESP32**: LM1117-3.3V voltage regulator (van 5V BEC)

#### Sensoren & Monitoring
- **Voltage divider**: 10kΩ + 4.7kΩ voor batterij monitoring (max 4.2V naar GPIO)
- **Capacitors**: 100µF op ESC power lines, 10µF op ESP32 voeding
- **Optioneel**: MPU6050 (gyro/accelerometer) voor stabilisatie

#### Pinout Vliegtuig ESP32
```
GPIO 25 -> ESC signaal (PWM 50Hz)
GPIO 26 -> Aileron servo (of elevator)
GPIO 27 -> Elevator servo (of rudder)
GPIO 14 -> Rudder servo (optioneel)
GPIO 34 -> Batterij voltage (ADC1, via divider)
GPIO 2  -> Status LED
I2C (GPIO 21/22) -> MPU6050 (optioneel)
5V van BEC -> LM1117-3.3V -> ESP32 3.3V pin
GND gedeeld met ESC/servos
```

### Controller

#### Hardware
- **Microcontroller**: ESP32 DevKit
- **Joysticks**: 2x analoge dual-axis joystick modules
  - Rechts: Throttle (Y-as), Rudder (X-as)
  - Links: Elevator (Y-as), Aileron (X-as)
- **Knoppen**: 
  - 1x Arm/Disarm toggle switch
  - 4x Trim knoppen (optioneel)
  - 1x Emergency stop
- **Display**: 0.96" OLED I2C (optioneel, voor telemetrie)
- **Voeding**: 2S LiPo of 9V batterij
- **Voltage regulator**: LM1117-3.3V (als van 9V of hoger)

#### Pinout Controller ESP32
```
GPIO 32 -> Throttle (ADC1)
GPIO 33 -> Rudder (ADC1)
GPIO 34 -> Elevator (ADC1)
GPIO 35 -> Aileron (ADC1)
GPIO 18 -> Arm/Disarm switch
GPIO 19 -> Emergency stop
GPIO 23 -> Trim+ knop
GPIO 22 -> Trim- knop
GPIO 2  -> Status LED
I2C (GPIO 21/22) -> OLED display (optioneel)
```

## Softwareontwerp

### ESP-NOW Communicatie

**Voordelen van ESP-NOW vs Bluetooth:**
- Lagere latency (<1ms vs 10-20ms)
- Betere range (tot 200m+ open veld)
- Geen pairing nodig
- Automatische retry op packet loss

#### Protocol Structuur
```cpp
// Packet van controller naar vliegtuig (20 bytes)
struct ControlPacket {
  uint8_t throttle;    // 0-180 (mapped naar PWM)
  uint8_t aileron;     // 0-180
  uint8_t elevator;    // 0-180
  uint8_t rudder;      // 0-180
  uint8_t armed;       // 0=disarmed, 1=armed
  uint8_t mode;        // 0=manual, 1=stabilized
  uint16_t checksum;   // CRC16 voor verificatie
};

// Telemetrie van vliegtuig naar controller (16 bytes)
struct TelemetryPacket {
  float battery_voltage;  // 4 bytes
  int16_t rssi;          // signaalsterkte
  uint8_t armed_status;  // 0/1
  uint8_t failsafe;      // 0/1
  uint16_t checksum;
};
```

### Vliegtuig Software (ESP32)

#### Hoofdfuncties
```cpp
// Setup
- ESP-NOW initialisatie
- ESC kalibratie sequentie
- Servo kalibratie (1000-2000µs)
- PWM channels setup (LEDC)
- Failsafe timer setup

// Loop
- ESP-NOW packet ontvangen
- Checksum verificatie
- Failsafe check (>1000ms geen data)
- PWM signalen updaten
- Batterij voltage lezen
- Telemetrie verzenden (10Hz)
```

#### Failsafe Gedrag
Bij verlies van signaal:
1. Throttle naar idle (0-5%)
2. Servos naar neutraal (90°)
3. Status LED knippert snel
4. Na 5 seconden: ESC disarm

#### ESC Kalibratie
```
1. Throttle stick maximaal
2. Power aan (hoor piepjes ESC)
3. Throttle stick minimaal
4. Wacht op bevestigingspiep
5. Klaar voor gebruik
```

### Controller Software (ESP32)

#### Hoofdfuncties
```cpp
// Setup
- ESP-NOW initialisatie met vliegtuig MAC
- Joystick kalibratie routine
- OLED display init (optioneel)

// Loop (50Hz update rate)
- Joystick waarden lezen (ADC)
- Dead-zone filtering (±5%)
- Exponential curve toepassen (voor fijnere besturing)
- Trim compensatie
- ARM check (veiligheidsinterlock)
- Packet versturen via ESP-NOW
- Telemetrie ontvangen en tonen
```

#### Joystick Kalibratie
```
- Opslaan min/max/center waarden in EEPROM
- Dode zone: ±5% rondom center
- Exponential curve: output = input^1.5 (zachter rondom center)
```

## Elektrisch Schema

### Vliegtuig Bedrading
```
3S LiPo (11.1V, XT60) 
  └─> ESC (rode/zwarte draden)
       ├─> Motor (3 draden)
       └─> BEC 5V output
            ├─> Servo's (rode draad = 5V)
            └─> LM1117-3.3V regulator
                 └─> ESP32 (3.3V pin)

Voltage Divider (batterij monitoring):
3S LiPo (+) ─[10kΩ]─┬─[4.7kΩ]─ GND
                    └─> GPIO34 (max 3.3V = 12.6V LiPo)
```

### Stroomverbruik (geschat)
- Motor (75% throttle): ~15A
- Servos (4x): ~0.5A
- ESP32: ~200mA
- **Totaal**: ~16A → 2200mAh / 16A = ~8 minuten vliegtijd

## 3D-Print / Bouwontwerp

### Opties voor Airframe
1. **Foam board** (aanbevolen voor eerste vliegtuig):
   - Lichtgewicht, goedkoop, gemakkelijk te repareren
   - Ontwerpen beschikbaar: "FT Flyer", "Versa Wing"
   
2. **3D-geprint**:
   - PETG voor structuur (20% infill)
   - PLA voor niet-structurele delen
   - Print in secties, gebruik carbon fiber rods voor versteviging

3. **Balsa hout**: Traditioneel, licht, sterke structuur

### Belangrijke Ontwerpprincipes
- **CG (Center of Gravity)**: 25-30% van vleugel chord
- **Motor thrust angle**: 2-3° naar beneden
- **Servo toegang**: Gemakkelijk bereikbaar voor onderhoud
- **Batterij positie**: Verstelbaar voor CG balancing
- **Duurzaam landingsgestel**: Absorbeert schokken

### Controller Behuizing
- Ergonomisch handgreep ontwerp (gamepad-stijl)
- Joysticks op natuurlijke duim-positie
- ARM switch onder vinger (veiligheid)
- OLED scherm zichtbaar boven joysticks
- Batterij compartiment aan achterzijde

## Veiligheidsfuncties

### Pre-flight Checklist
1. ✓ Batterij volledig geladen (>11.1V)
2. ✓ Propeller stevig gemonteerd
3. ✓ Servos bewegen in correcte richting
4. ✓ Range test (50m lopen met controller)
5. ✓ Failsafe test (controller uit, servos naar neutraal)
6. ✓ CG controle (vliegtuig balanceert op 25-30% chord)

### Arming Interlock
- Throttle moet op 0% staan
- ARM switch moet ingedrukt worden
- LED geeft bevestiging
- ESC piept ter bevestiging

### Emergency Procedures
- **Loss of signal**: Failsafe activeert automatisch
- **Low battery**: Waarschuwing op controller, land onmiddellijk
- **Loss of control**: Activeer disarm switch

## Software Stack

### ESP32 Libraries
```cpp
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>  // of gebruik LEDC PWM
#include <Wire.h>        // voor I2C (OLED/MPU6050)
#include <Preferences.h> // voor EEPROM storage
```

### Ontwikkelomgeving
- Arduino IDE met ESP32 board support
- Of PlatformIO (aanbevolen voor grotere projecten)

## Toekomstige Uitbreidingen

### Fase 1 (Huidige Plan)
- ✓ Basis RC besturing
- ✓ Telemetrie
- ✓ Failsafe

### Fase 2
- MPU6050 stabilisatie (zelf-nivellerend)
- Barometer voor hoogte-hold
- Betere batterij monitoring (per cel)

### Fase 3
- GPS positie hold
- Return-to-home functie
- Mission planning

### Fase 4
- FPV camera systeem
- OSD (On-Screen Display) voor telemetrie
- Gimbal stabilisatie

## Kostenraming (geschat)
- ESP32 (2x): €15
- Perfboards (2x): €5
- 3S LiPo 2200mAh: €25
- Motor + ESC: €30
- Servos (4x): €15
- Joysticks + knoppen: €10
- Voltage regulators + componenten: €10
- Foam board / printmaterialen: €20
- **Totaal**: ~€130

## Resources
- ESP-NOW documentatie: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
- FliteTest (foam board vliegtuigen): https://www.flitetest.com/
- RC Groups forum: https://www.rcgroups.com/
- PlatformIO ESP32: https://docs.platformio.org/en/latest/platforms/espressif32.html
