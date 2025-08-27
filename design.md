# RC Vliegtuig - Technisch Ontwerp

## Systeemarchitectuur

### Overzicht
Het RC vliegtuigproject bestaat uit twee hoofdcomponenten:
1. **Vliegtuig**: Met een microcontroller die de motoren en servo's aanstuurt
2. **Controller**: Met een microcontroller die gebruikersinput verwerkt en verzendt naar het vliegtuig

De twee componenten communiceren draadloos via Bluetooth-modules, waarbij een betrouwbaar protocol met foutdetectie wordt gebruikt.

### Communicatiediagram
```
+--------------------+                      +--------------------+
|    CONTROLLER      |                      |      VLIEGTUIG     |
|                    |                      |                    |
|  +-------------+   |     Bluetooth        |  +-------------+   |
|  | Arduino     |<--+--------------------->+--| Arduino     |   |
|  | Joysticks   |   |     Protocool        |  | ESC & Motor |   |
|  | Knoppen     |   |                      |  | Servo's     |   |
|  +-------------+   |                      |  | Accu        |   |
+--------------------+                      +--------------------+
```

## Hardwareontwerp

### Vliegtuig
- **Microcontroller**: Arduino Nano (compact en licht)
- **Bluetooth**: HC-05 module (master mode)
- **Motor**: 2212 980KV borstelloze motor
- **ESC**: 30A ESC met BEC (Battery Elimination Circuit)
- **Servo's**: 2x 9g micro servo's (1 voor aileron, 1 voor elevator)
- **Batterij**: 2S LiPo 7.4V 1500mAh
- **Frame**: 3D-geprint PLA of PETG (voor sterkte en licht gewicht)
- **Spanningsdeler**: Weerstandsnetwerk voor batterijmonitoring
- **Aansluitingen**:
  - ESC signaal -> Pin 9 (PWM)
  - Aileron servo -> Pin 10 (PWM)
  - Elevator servo -> Pin 11 (PWM)
  - Bluetooth RX -> Pin 2
  - Bluetooth TX -> Pin 3
  - Batterij monitoring -> A0

### Controller
- **Microcontroller**: Arduino Nano
- **Bluetooth**: HC-06 module (slave mode)
- **Joysticks**: 2 analoge joysticks (potentiometers)
- **Knoppen**: 5 drukknoppen (arm/disarm + trim)
- **LED**: Status-LED voor verbinding en batterij
- **Batterij**: 9V batterij of LiPo accu
- **Behuizing**: 3D-geprint of aangepaste gamecontroller
- **Aansluitingen**:
  - Throttle -> A0
  - Aileron -> A1
  - Elevator -> A2
  - Arm/Disarm knop -> Pin 4
  - Trim knoppen -> Pin 5-8
  - Bluetooth RX -> Pin 2
  - Bluetooth TX -> Pin 3
  - Status LED -> Pin 13

## Softwareontwerp

### Communicatieprotocol
Het communicatieprotocol tussen controller en vliegtuig is ontworpen voor betrouwbaarheid:

1. **Besturingscommando's** (van controller naar vliegtuig):
   - Format: `T:[throttle],A:[aileron],E:[elevator],CHK:[checksum]`
   - Voorbeeld: `T:90,A:110,E:85,CHK:1234`

2. **Statusberichten** (van vliegtuig naar controller):
   - Format: `STATUS:[ARMED/DISARMED],BAT:[voltage],TRIM:A:[aileronTrim],E:[elevatorTrim]`
   - Voorbeeld: `STATUS:ARMED,BAT:7.6,TRIM:A:5,E:-2`

3. **Speciale commando's**:
   - `ARM` / `DISARM` - voor het activeren/deactiveren van de motor
   - `TRIM:A:[waarde],E:[waarde]` - voor het aanpassen van trim-waarden

4. **Foutdetectie**:
   - Checksum: Som van ASCII-waarden van alle karakters in het bericht
   - Timeout-detectie: Failsafe-activering als er geen commando's worden ontvangen

### Vliegtuig Software Features
- ESC-initialisatie en arm/disarm-functionaliteit
- Servo-besturing met trim-compensatie
- Failsafe-activering bij communicatieverlies
- Batterijmonitoring met waarschuwingen
- EEPROM-opslag voor trim-instellingen

### Controller Software Features
- Joystick-kalibratie en dode zone voor stabiele besturing
- Trim-aanpassingen via drukknoppen
- Verbindingsstatusmonitoring
- Batterijstatusweergave van het vliegtuig
- Commando's met checksum voor foutdetectie

## 3D-printontwerp
Het ontwerp van het vliegtuig is geoptimaliseerd voor 3D-printen:

### Belangrijke ontwerpelementen
1. **Rompontwerp**: Aerodynamisch, met compartimenten voor:
   - Batterij (gemakkelijk toegankelijk voor verwisselen)
   - Elektronica (beschermd maar met koeling)
   - Servo's (geïntegreerd in de vleugels)

2. **Vleugels**:
   - Geprint in secties die samengevoegd worden
   - Geïntegreerde kanalen voor servo-kabels
   - Verstevigde verbindingspunten
   - Beweegbare aileron-onderdelen

3. **Stabilisatoren**:
   - Horizontaal stabilisatievlak met beweegbare elevator
   - Verticaal stabilisatievlak voor richting

4. **Motor mount**:
   - Verstevigd ontwerp voor het vasthouden van de motor
   - Instelbare hoek voor thrust-configuratie

5. **Controller behuizing**:
   - Ergonomisch handgreep-ontwerp
   - Knoppen en joysticks positionering voor optimale bediening
   - Batterijcompartiment
   - Zichtbare LED voor status

### Materiaal- en printsuggesties
- PETG voor structurele onderdelen (sterker dan PLA, minder warping dan ABS)
- 20-30% infill voor stevigheid met minimaal gewicht
- Verstevigde verbindingspunten voor hoge stressgebieden
- Lichtgewicht honingraatstructuur waar mogelijk

## Toekomstige Uitbreidingen
1. **Sensoren toevoegen**:
   - Gyroscoop en accelerometer voor stabilisatie
   - Barometer voor hoogtemeting
   - GPS voor positiebepaling

2. **Camera-integratie**:
   - FPV (First Person View) camera
   - Stabilisatie-gimbal

3. **Verbeterde Controller**:
   - LCD-scherm voor telemetrie-weergave
   - Meer kanalen voor extra functies
   - Instelbare vliegmodi
