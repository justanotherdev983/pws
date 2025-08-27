# RC Vliegtuig Project

## Projectoverzicht
Dit project betreft het ontwerpen, bouwen en programmeren van een op afstand bestuurbaar vliegtuig (RC vliegtuig) als onderdeel van ons profielwerkstuk voor informatica. Het project omvat het 3D-printen van het vliegtuigontwerp, het solderen van de elektronica, en het programmeren van twee microcontrollers voor zowel het vliegtuig als de controller.

## Functionaliteiten
- Op afstand bestuurbaar vliegtuig via een zelfgemaakte controller
- Bluetooth communicatie tussen controller en vliegtuig
- Besturing van motoren (throttle) en servo's (aileron en elevator)
- Realtime feedback van batterijstatus
- Veiligheidsfuncties zoals failsafe bij verbindingsverlies
- Trim-instelling voor fijnafstelling van besturing

## Hardware Vereisten
### Vliegtuig
- Arduino of compatibele microcontroller
- Bluetooth module (HC-05/HC-06)
- ESC (Electronic Speed Controller) voor de motor
- Borstelloze motor
- Servo's voor aileron en elevator
- LiPo batterij (2S of 3S)
- Voltage divider voor batterijmonitoring
- 3D-geprint frame en onderdelen

### Controller
- Arduino of compatibele microcontroller
- Bluetooth module (HC-05/HC-06)
- Joysticks (potentiometers) voor besturing
- Drukknoppen voor arm/disarm en trim
- LED voor statusweergave
- Batterij voor de controller

## Software
- `plane.c`: Code voor de microcontroller in het vliegtuig
- `controller.c`: Code voor de microcontroller in de afstandsbediening
- Beide programma's communiceren via een betrouwbaar protocol met checksum-verificatie

## Installatie en Gebruik
1. 3D-print alle benodigde vliegtuigonderdelen
2. Assembleer het vliegtuig en controller volgens het ontwerp
3. Soldeer de elektronische componenten
4. Upload de respectievelijke code naar de microcontrollers
5. Kalibreer de joysticks en servo's
6. Test de verbinding en alle functies

## Veiligheidsmaatregelen
- Arm/disarm functie voorkomt onbedoeld opstarten van de motor
- Failsafe-functie brengt het vliegtuig naar een veilige stand bij verbindingsverlies
- Batterijmonitoring met waarschuwing bij lage spanning
- Automatische uitschakeling bij kritiek lage batterijspanning

## Toekomstige Verbeteringen
- GPS-modules voor locatiemonitoring
- Telemetrie voor uitgebreidere vliegdata
- Camera voor FPV (First Person View) vliegen
- Automatische stabilisatie
- Verbeterd vliegtuigontwerp voor betere aerodynamica


## Licentie
Dit project is gemaakt als onderdeel van een schoolopdracht en is beschikbaar voor educatieve doeleinden.
