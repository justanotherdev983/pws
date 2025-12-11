# RC Vliegtuig PWS 


## Functionaliteiten
- Code voor op afstand bestuurbaar vliegtuig via een zelfgemaakte controller
- Bluetooth communicatie tussen controller en vliegtuig
- Besturing van motoren (throttle) en servo's (aileron en elevator)

## Hardware Vereisten
### Vliegtuig
- ESP32 dev kit met BT module 
- ESC (Electronic Speed Controller) voor de motor
- Borstelloze motor
- Servo's voor aileron en elevator
- LiPo batterij 
- 3D-geprint frame en onderdelen, te vinden in code bij src/ als python of direct FreeCAD/

### Controller
- ESP32 dev kit met BT module 
- 2 Joysticks (potentiometers) 
- Drukknoppen voor arm/disarm 
- Batterij voor de controller

## Software
- `esp/src/plane.c`: Code voor de microcontroller in het vliegtuig
- `esp/src/controller.c`: Code voor de microcontroller in de afstandsbediening

## Installatie en Gebruik
1. 3D-print alle benodigde vliegtuigonderdelen
2. zet het vliegtuig en controller volgens het ontwerp inelkaar
3. Soldeer de elektronische componenten
4. Upload de code naar de microcontrollers
5. Kalibreer de joysticks en servo's
6. Test de verbinding en alle functies

# Compile en run op ESP32
```bash
python3 -m venv .venv

source ./.venv/bin/activate

pip install platformio
sudo chmod 666 /dev/ttyUSB0 
## Voor vliegtuig
pio run -e plane --target upload --upload-port /dev/ttyUSB0
## Voor controller
pio run -e controller --target upload --upload-port /dev/ttyUSB0

```

# Run python voor FreeCAD
plak de code in de python console en wacht tot het klaar is
