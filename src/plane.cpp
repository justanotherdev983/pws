/**
 * RC Vliegtuig - Onboard Controller Code
 * 
 * Dit programma bestuurt een RC vliegtuig met een Arduino microcontroller.
 * Functies:
 * - Bluetooth communicatie met de controller
 * - Aansturing van ESC (motor) en servo's (aileron, elevator)
 * - Batterij monitoring
 * - Failsafe bij verbindingsverlies
 * - Trim instellingen opgeslagen in EEPROM
 */

#include <Servo.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

// Definieer pins
#define ESC_PIN 9             // PWM pin voor Electronic Speed Controller
#define SERVO_AILERON_PIN 10  // PWM pin voor aileron servo
#define SERVO_ELEVATOR_PIN 11 // PWM pin voor elevator servo
#define BT_RX 2               // Bluetooth RX pin
#define BT_TX 3               // Bluetooth TX pin
#define BATTERY_PIN A0        // Analoge pin voor batterij voltage meting

// Maak servo objecten
Servo ESC;
Servo aileronServo;
Servo elevatorServo;

// Bluetooth communicatie
SoftwareSerial bluetooth(BT_RX, BT_TX);

// Variabelen voor besturing
int throttle = 0;             // 0-180 voor ESC
int aileron = 90;             // 90 is neutrale positie
int elevator = 90;            // 90 is neutrale positie

// Trim waarden (worden opgeslagen in EEPROM)
int aileronTrim = 0;          // -30 tot +30
int elevatorTrim = 0;         // -30 tot +30

// Failsafe waarden
const int FAILSAFE_THROTTLE = 0;  // Motor uit bij verlies van verbinding
const int FAILSAFE_AILERON = 90;  // Neutrale positie
const int FAILSAFE_ELEVATOR = 90; // Neutrale positie

// Timers
unsigned long lastCommandTime = 0;
const unsigned long failsafeTimeout = 1000; // 1 seconde zonder commando's = failsafe

// Batterij monitoring
float batteryVoltage = 0;
const float LOW_VOLTAGE_THRESHOLD = 7.0;  // Voor 2S LiPo (aanpassen voor 3S naar ~10.5V)
const float VOLTAGE_DIVIDER_RATIO = 3.0;  // Aanpassen op basis van je voltage divider circuit

// ESC arming status
bool escArmed = false;

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  
  // Initialiseer servo's
  ESC.attach(ESC_PIN);
  aileronServo.attach(SERVO_AILERON_PIN);
  elevatorServo.attach(SERVO_ELEVATOR_PIN);
  
  // Laad trim waarden uit EEPROM
  loadTrimsFromEEPROM();
  
  // ESC initialisatie (motor uit)
  initESC();
  
  // Zet beginpositie
  setControls(0, 90, 90);
  
  Serial.println("Vliegtuig systeem gestart");
  
  // Stuur status naar controller
  sendStatus();
}

void loop() {
  // Controleer op nieuwe commando's
  if (bluetooth.available() > 0) {
    String command = bluetooth.readStringUntil('\n');
    if (validateCommand(command)) {
      parseCommand(command);
      lastCommandTime = millis();
    }
  }
  
  // Check voor failsafe (verbinding verloren)
  if (millis() - lastCommandTime > failsafeTimeout) {
    activateFailsafe();
  }
  
  // Meet batterij voltage elke seconde
  static unsigned long lastBatteryCheck = 0;
  if (millis() - lastBatteryCheck > 1000) {
    checkBattery();
    lastBatteryCheck = millis();
  }
  
  // Stuur periodiek status terug naar controller
  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate > 2000) {
    sendStatus();
    lastStatusUpdate = millis();
  }
}

// Commando validatie met eenvoudige checksum
bool validateCommand(String command) {
  if (command.indexOf("CHK:") == -1) return false;
  
  int chkIndex = command.indexOf("CHK:");
  String dataSection = command.substring(0, chkIndex);
  String checksumStr = command.substring(chkIndex + 4);
  
  // Bereken eenvoudige checksum (som van ASCII-waarden)
  int calculatedChecksum = 0;
  for (int i = 0; i < dataSection.length(); i++) {
    calculatedChecksum += dataSection.charAt(i);
  }
  
  int receivedChecksum = checksumStr.toInt();
  return (calculatedChecksum == receivedChecksum);
}

void parseCommand(String command) {
  // De volledige string heeft nu dit formaat: "T:val,A:val,E:val,CHK:checksum"
  
  // Command types
  if (command.indexOf("ARM") != -1) {
    armESC();
    return;
  }
  
  if (command.indexOf("DISARM") != -1) {
    disarmESC();
    return;
  }
  
  if (command.indexOf("TRIM") != -1) {
    // Trim commando: "TRIM:A:5,E:-3"
    parseTrimCommand(command);
    return;
  }
  
  // Standaard besturingscommando
  int tIndex = command.indexOf("T:");
  int aIndex = command.indexOf("A:");
  int eIndex = command.indexOf("E:");
  int chkIndex = command.indexOf("CHK:");
  
  int newThrottle = throttle;
  int newAileron = aileron;
  int newElevator = elevator;
  
  if (tIndex != -1) {
    int commaIndex = command.indexOf(',', tIndex);
    if (commaIndex == -1) commaIndex = chkIndex;
    newThrottle = command.substring(tIndex + 2, commaIndex).toInt();
  }
  
  if (aIndex != -1) {
    int commaIndex = command.indexOf(',', aIndex);
    if (commaIndex == -1) commaIndex = chkIndex;
    newAileron = command.substring(aIndex + 2, commaIndex).toInt();
  }
  
  if (eIndex != -1) {
    int commaIndex = command.indexOf(',', eIndex);
    if (commaIndex == -1) commaIndex = chkIndex;
    newElevator = command.substring(eIndex + 2, commaIndex).toInt();
  }
  
  // Toepassen van de nieuwe controlewaarden (alleen als ESC armed is voor throttle)
  setControls(escArmed ? newThrottle : 0, newAileron, newElevator);
}

void parseTrimCommand(String command) {
  int aIndex = command.indexOf("A:");
  int eIndex = command.indexOf("E:");
  
  if (aIndex != -1) {
    int commaIndex = command.indexOf(',', aIndex);
    if (commaIndex == -1) commaIndex = command.length();
    aileronTrim = command.substring(aIndex + 2, commaIndex).toInt();
  }
  
  if (eIndex != -1) {
    int commaIndex = command.indexOf(',', eIndex);
    if (commaIndex == -1) commaIndex = command.length();
    elevatorTrim = command.substring(eIndex + 2, commaIndex).toInt();
  }
  
  // Sla nieuwe trim waarden op
  saveTrimsToEEPROM();
  
  // Update besturing met nieuwe trim
  setControls(throttle, aileron, elevator);
}

void setControls(int newThrottle, int newAileron, int newElevator) {
  // Begrens de waarden
  throttle = constrain(newThrottle, 0, 180);
  aileron = constrain(newAileron + aileronTrim, 0, 180);
  elevator = constrain(newElevator + elevatorTrim, 0, 180);
  
  // Pas servo posities toe
  ESC.write(throttle);
  aileronServo.write(aileron);
  elevatorServo.write(elevator);
}

void activateFailsafe() {
  Serial.println("FAILSAFE GEACTIVEERD - Verbinding verloren");
  
  // Zet controles naar veilige waarden
  setControls(FAILSAFE_THROTTLE, FAILSAFE_AILERON, FAILSAFE_ELEVATOR);
  escArmed = false;
}

void initESC() {
  // ESC kalibratie/initialisatie sequentie
  ESC.write(0);
  delay(1000);
  escArmed = false;
  Serial.println("ESC geïnitialiseerd");
}

void armESC() {
  if (!escArmed) {
    escArmed = true;
    Serial.println("ESC ARMED");
    
    // Bevestiging naar controller
    bluetooth.println("STATUS:ARMED");
  }
}

void disarmESC() {
  if (escArmed) {
    escArmed = false;
    throttle = 0;
    ESC.write(0);
    Serial.println("ESC DISARMED");
    
    // Bevestiging naar controller
    bluetooth.println("STATUS:DISARMED");
  }
}

void checkBattery() {
  int rawValue = analogRead(BATTERY_PIN);
  
  // Converteer analoge waarde naar voltage
  // Arduino analoge input: 0-1023 = 0-5V, met voltage divider
  float voltage = (rawValue / 1023.0) * 5.0 * VOLTAGE_DIVIDER_RATIO;
  batteryVoltage = voltage;
  
  // Check voor lage spanning
  if (voltage < LOW_VOLTAGE_THRESHOLD) {
    Serial.println("WAARSCHUWING: Lage batterijspanning: " + String(voltage, 1) + "V");
    
    // Stuur waarschuwing naar controller
    bluetooth.println("ALERT:LOWBAT:" + String(voltage, 1));
    
    // Extra veiligheid: als de batterij ECHT laag is, disarm de ESC
    if (voltage < (LOW_VOLTAGE_THRESHOLD - 0.5)) {
      disarmESC();
    }
  }
}

void loadTrimsFromEEPROM() {
  // EEPROM adres 0 = aileron trim, adres 1 = elevator trim
  aileronTrim = EEPROM.read(0) - 127;  // Offset om -127 tot +128 bereik te krijgen
  elevatorTrim = EEPROM.read(1) - 127;
  
  Serial.println("Trims geladen: A:" + String(aileronTrim) + " E:" + String(elevatorTrim));
}

void saveTrimsToEEPROM() {
  // Bewaar met offset zodat negatieve waarden ook opgeslagen kunnen worden
  EEPROM.write(0, aileronTrim + 127);
  EEPROM.write(1, elevatorTrim + 127);
  
  Serial.println("Trims opgeslagen: A:" + String(aileronTrim) + " E:" + String(elevatorTrim));
}

void sendStatus() {
  String status = "STATUS:";
  
  // Voeg ESC status toe
  status += escArmed ? "ARMED," : "DISARMED,";
  
  // Voeg batterij voltage toe
  status += "BAT:" + String(batteryVoltage, 1) + ",";
  
  // Voeg huidige trims toe
  status += "TRIM:A:" + String(aileronTrim) + ",E:" + String(elevatorTrim);
  
  bluetooth.println(status);
}
