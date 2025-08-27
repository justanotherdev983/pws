// CONTROLLER CODE (remote microcontroller)
#include <SoftwareSerial.h>
#include <EEPROM.h>

// Definieer pins
#define THROTTLE_PIN A0
#define AILERON_PIN A1
#define ELEVATOR_PIN A2
#define ARM_BUTTON_PIN 4
#define TRIM_UP_PIN 5
#define TRIM_DOWN_PIN 6
#define TRIM_LEFT_PIN 7
#define TRIM_RIGHT_PIN 8
#define BT_RX 2
#define BT_TX 3
#define BATTERY_LED_PIN 13

#define EEPROM_THROTTLE_MIN_ADDR 0
#define EEPROM_THROTTLE_MAX_ADDR 4
#define EEPROM_AILERON_CENTER_ADDR 8
#define EEPROM_ELEVATOR_CENTER_ADDR 12

// Bluetooth communicatie
SoftwareSerial bluetooth(BT_RX, BT_TX);

// Variabelen voor besturing
int throttle = 0;
int aileron = 90;
int elevator = 90;

// Trim waarden
int aileronTrim = 0;
int elevatorTrim = 0;

// Joystick kalibratie
int throttleMin = 0;
int throttleMax = 1023;
int aileronCenter = 512;
int elevatorCenter = 512;

// Status variabelen
bool isArmed = false;
float planeVoltage = 0.0;
unsigned long lastReceivedTime = 0;
bool connectionLost = true;

// Timers
unsigned long lastTransmission = 0;
const int transmitInterval = 100;  // 10Hz transmissie
unsigned long lastButtonCheck = 0;
const int buttonCheckInterval = 50;  // 20Hz button checks

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  
  // Button input pins
  pinMode(ARM_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TRIM_UP_PIN, INPUT_PULLUP);
  pinMode(TRIM_DOWN_PIN, INPUT_PULLUP);
  pinMode(TRIM_LEFT_PIN, INPUT_PULLUP);
  pinMode(TRIM_RIGHT_PIN, INPUT_PULLUP);
  
  // LED output
  pinMode(BATTERY_LED_PIN, OUTPUT);
  
  // Laad kalibratie als die bestaat
  loadCalibration();
  
  // Wacht op bluetooth connectie
  Serial.println("Controller gestart. Verbinding maken...");
  
  // Begin niet in armed status
  isArmed = false;
}

void loop() {
  // Controleer knoppen
  if (millis() - lastButtonCheck > buttonCheckInterval) {
    checkButtons();
    lastButtonCheck = millis();
  }
  
  // Lees joystick waarden
  readControls();
  
  // Stuur commando's met intervallen
  if (millis() - lastTransmission > transmitInterval) {
    sendControlCommand();
    lastTransmission = millis();
  }
  
  // Controleer voor inkomende status updates van het vliegtuig
  if (bluetooth.available() > 0) {
    String message = bluetooth.readStringUntil('\n');
    parseStatus(message);
    lastReceivedTime = millis();
    
    // We hebben bericht ontvangen, verbinding is OK
    if (connectionLost) {
      connectionLost = false;
      Serial.println("Verbinding hersteld");
    }
  }
  
  // Check voor verloren verbinding
  if (millis() - lastReceivedTime > 5000) {
    if (!connectionLost) {
      connectionLost = true;
      Serial.println("WAARSCHUWING: Verbinding verloren");
      // Hier kun je een buzzer of LED waarschuwing toevoegen
    }
  }
  
  // Laat batterij status zien (knipperend bij laag voltage)
  updateBatteryLED();
}

void parseStatus(String message) {
  // Status bericht format: "STATUS:ARMED/DISARMED,BAT:10.4,TRIM:A:5,E:-2"
  if (message.indexOf("STATUS:") != -1) {
    // Parse armed status
    if (message.indexOf("ARMED") != -1) {
      isArmed = true;
    } else if (message.indexOf("DISARMED") != -1) {
      isArmed = false;
    }
    
    // Parse batterij voltage
    int batIndex = message.indexOf("BAT:");
    if (batIndex != -1) {
      int commaIndex = message.indexOf(',', batIndex);
      if (commaIndex == -1) commaIndex = message.length();
      planeVoltage = message.substring(batIndex + 4, commaIndex).toFloat();
    }
    
    // Parse trims (ter bevestiging)
    int trimAIndex = message.indexOf("TRIM:A:");
    int trimEIndex = message.indexOf("E:");
    
    if (trimAIndex != -1 && trimEIndex != -1) {
      int commaIndex = message.indexOf(',', trimAIndex + 7);
      if (commaIndex == -1) commaIndex = message.length();
      aileronTrim = message.substring(trimAIndex + 7, trimEIndex - 1).toInt();
      
      elevatorTrim = message.substring(trimEIndex + 2).toInt();
    }
    
    // Debug print
    Serial.println("Status: " + (isArmed ? String("ARMED") : String("DISARMED")) + 
                  " Batt: " + String(planeVoltage, 1) + "V" + 
                  " Trim A:" + String(aileronTrim) + " E:" + String(elevatorTrim));
  }
  
  // Alert berichten
  if (message.indexOf("ALERT:") != -1) {
    if (message.indexOf("LOWBAT:") != -1) {
      // Handle lage batterij waarschuwing
      int valueStart = message.indexOf("LOWBAT:") + 7;
      planeVoltage = message.substring(valueStart).toFloat();
      
      Serial.println("WAARSCHUWING: Lage Batterij: " + String(planeVoltage, 1) + "V");
      // Hier zou je een buzzer kunnen activeren
    }
  }
}

void readControls() {
  // Lees raw joystick waarden
  int rawThrottle = analogRead(THROTTLE_PIN);
  int rawAileron = analogRead(AILERON_PIN);
  int rawElevator = analogRead(ELEVATOR_PIN);
  
  // Pas kalibratie toe 
  throttle = map(constrain(rawThrottle, throttleMin, throttleMax), throttleMin, throttleMax, 0, 180);
  
  // Centreer en schaal aileron en elevator
  int aileronOffset = rawAileron - aileronCenter;
  int elevatorOffset = rawElevator - elevatorCenter;
  
  // Dode zone toepassen om trillingen te voorkomen
  const int DEAD_ZONE = 20;
  if (abs(aileronOffset) < DEAD_ZONE) aileronOffset = 0;
  if (abs(elevatorOffset) < DEAD_ZONE) elevatorOffset = 0;
  
  // Schaal naar servo bereik (90±45)
  aileron = 90 + map(aileronOffset, -512, 512, -45, 45);
  elevator = 90 + map(elevatorOffset, -512, 512, -45, 45);
  
  // Begrens de waarden voor zekerheid
  throttle = constrain(throttle, 0, 180);
  aileron = constrain(aileron, 45, 135);
  elevator = constrain(elevator, 45, 135);
}

void checkButtons() {
  // ARM/DISARM toggle
  static bool armButtonLast = HIGH;
  bool armButtonCurrent = digitalRead(ARM_BUTTON_PIN);
  
  if (armButtonLast == HIGH && armButtonCurrent == LOW) {
    // Button ingedrukt - toggle arm status
    if (isArmed) {
      sendCommand("DISARM");
    } else {
      sendCommand("ARM");
    }
  }
  armButtonLast = armButtonCurrent;
  
  // Trim knoppen
  if (digitalRead(TRIM_UP_PIN) == LOW) {
    elevatorTrim++;
    sendTrimCommand();
    delay(100); // Kleine vertraging om niet te snel te incrementeren
  }
  
  if (digitalRead(TRIM_DOWN_PIN) == LOW) {
    elevatorTrim--;
    sendTrimCommand();
    delay(100);
  }
  
  if (digitalRead(TRIM_LEFT_PIN) == LOW) {
    aileronTrim--;
    sendTrimCommand();
    delay(100);
  }
  
  if (digitalRead(TRIM_RIGHT_PIN) == LOW) {
    aileronTrim++;
    sendTrimCommand();
    delay(100);
  }
  
  // Begrens trim waarden
  aileronTrim = constrain(aileronTrim, -30, 30);
  elevatorTrim = constrain(elevatorTrim, -30, 30);
}

void sendControlCommand() {
  String command = "T:" + String(throttle) + ",A:" + String(aileron) + ",E:" + String(elevator);
  
  // Voeg checksum toe
  int checksum = 0;
  for (int i = 0; i < command.length(); i++) {
    checksum += command.charAt(i);
  }
  command += ",CHK:" + String(checksum);
  
  // Verstuur naar vliegtuig
  bluetooth.println(command);
  
  // Debug output (maar niet elke keer om serial spam te voorkomen)
  static unsigned long lastDebugOutput = 0;
  if (millis() - lastDebugOutput > 1000) {  // Eens per seconde
    Serial.println("TX: " + command);
    lastDebugOutput = millis();
  }
}

void sendCommand(String cmd) {
  bluetooth.println(cmd);
  Serial.println("Command verzonden: " + cmd);
}

void sendTrimCommand() {
  String trimCmd = "TRIM:A:" + String(aileronTrim) + ",E:" + String(elevatorTrim);
  sendCommand(trimCmd);
  Serial.println("Trim aangepast: A:" + String(aileronTrim) + " E:" + String(elevatorTrim));
}

void loadCalibration() {
  // In een echte implementatie zou je hier de joystick kalibratie uit EEPROM laden
  // Voor nu gebruiken we standaardwaarden
  throttleMin = 0;
  throttleMax = 1023;
  aileronCenter = 512;
  elevatorCenter = 512;
}

void updateBatteryLED() {
  if (planeVoltage <= 0) {
    // Geen voltage info ontvangen
    digitalWrite(BATTERY_LED_PIN, connectionLost ? LOW : HIGH);
  } else if (planeVoltage < 7.0) {  // Voor 2S LiPo
    // Lage batterij - knipperen
    digitalWrite(BATTERY_LED_PIN, (millis() / 250) % 2 == 0 ? HIGH : LOW);
  } else {
    // Batterij OK - LED constant aan
    digitalWrite(BATTERY_LED_PIN, HIGH);
  }
}

// Functie voor het kalibreren van de joysticks
// Deze zou je kunnen aanroepen bij een speciale knoppencombinatie
void calibrateJoysticks() {
  Serial.println("Kalibratie gestart. Zet alle sticks in neutrale positie");
  delay(2000);
  
  // Lees neutrale posities
  aileronCenter = analogRead(AILERON_PIN);
  elevatorCenter = analogRead(ELEVATOR_PIN);
  
  Serial.println("Beweeg throttle helemaal naar beneden (minimum)");
  delay(2000);
  throttleMin = analogRead(THROTTLE_PIN);
  
  Serial.println("Beweeg throttle helemaal naar boven (maximum)");
  delay(2000);
  throttleMax = analogRead(THROTTLE_PIN);
  
  Serial.println("Kalibratie voltooid:");
  Serial.println("Throttle min: " + String(throttleMin));
  Serial.println("Throttle max: " + String(throttleMax));
  Serial.println("Aileron center: " + String(aileronCenter));
  Serial.println("Elevator center: " + String(elevatorCenter));

  saveCalibration();
}

void saveCalibration() {
  EEPROM.put(EEPROM_THROTTLE_MIN_ADDR, throttleMin);
  EEPROM.put(EEPROM_THROTTLE_MAX_ADDR, throttleMax);
  EEPROM.put(EEPROM_AILERON_CENTER_ADDR, aileronCenter);
  EEPROM.put(EEPROM_ELEVATOR_CENTER_ADDR, elevatorCenter);
  Serial.println("Calibration saved to EEPROM");
}

void loadCalibration() {
  EEPROM.get(EEPROM_THROTTLE_MIN_ADDR, throttleMin);
  EEPROM.get(EEPROM_THROTTLE_MAX_ADDR, throttleMax);
  EEPROM.get(EEPROM_AILERON_CENTER_ADDR, aileronCenter);
  EEPROM.get(EEPROM_ELEVATOR_CENTER_ADDR, elevatorCenter);
  Serial.println("Calibration loaded from EEPROM");
  // Add a check:  If EEPROM is empty, load defaults instead
  if (throttleMin == 0 && throttleMax == 0 && aileronCenter == 0 && elevatorCenter == 0) {
    Serial.println("EEPROM empty, using default calibration");
    throttleMin = 0;
    throttleMax = 1023;
    aileronCenter = 512;
    elevatorCenter = 512;
  }
}
