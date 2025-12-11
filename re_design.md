# RC Vliegtuig - Technisch Ontwerp (ESP32)

## Systeemarchitectuur

### Overzicht
Het RC vliegtuigproject bestaat uit twee hoofdcomponenten:
1. **Vliegtuig**: Met ESP32 microcontroller die de motor (via ESC) en servo's aanstuurt
2. **Controller**: Met ESP32 microcontroller die gebruikersinput verwerkt en verzendt


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


### Ontwikkelomgeving
- PlatformIO via terminal 
