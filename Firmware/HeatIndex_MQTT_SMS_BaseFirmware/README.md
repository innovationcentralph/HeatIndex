# ESP32 CLI Command Interface

This project provides a Serial-based Command Line Interface (CLI) for configuring and controlling an ESP32 device. It uses AT-style commands for ease of use and compatibility with other tools.

---

## 🚀 Getting Started

Connect to the ESP32 device via Serial (e.g., using Arduino Serial Monitor or a terminal tool like PuTTY) and send any of the following commands terminated with `\r` or `\n`.

---

## 🧾 Command List

### 🔧 General

| Command              | Description                                |
|----------------------|--------------------------------------------|
| `AT+HELP` or `?`     | Show basic list of commands                |
| `AT+README`          | Show detailed usage information (optional) |
| `AT+RESTART`         | Restart the ESP32 device                   |

---

### 🔐 Device Identification

| Command             | Description                        |
|---------------------|------------------------------------|
| `AT+SETESN=<esn>`   | Set device ESN (e.g., `12345678`)  |
| `AT+ESN?`           | Show currently stored ESN          |

---

### 📡 MQTT Configuration

| Command                                      | Description                        |
|----------------------------------------------|------------------------------------|
| `AT+SETMQTT=server,user,pass,port`           | Configure MQTT connection settings |
| `AT+MQTT?`                                   | Show current MQTT configuration    |

> Example:
> ```
> AT+SETMQTT=mqtt.example.com,myuser,mypass,1883
> ```

---

### ❤️ Heartbeat Configuration

| Command                         | Description                         |
|----------------------------------|-------------------------------------|
| `AT+SET_HB_INTERVAL=<ms>`       | Set heartbeat interval (in ms)      |
| `AT+GET_HB_INTERVAL`            | Show current heartbeat interval     |

---

### 📶 WiFi Configuration

| Command                          | Description                         |
|----------------------------------|-------------------------------------|
| `AT+SETWIFI=ssid,password`       | Save WiFi credentials to EEPROM     |
| `AT+WIFI?`                       | Display stored WiFi credentials     |

---

### 🌐 Network Mode

| Command                     | Description                         |
|-----------------------------|-------------------------------------|
| `AT+SETNET=wifi`            | Use WiFi only                       |
| `AT+SETNET=gsm`             | Use GSM only                        |
| `AT+NET?`                   | Display current network mode        |

---

## 💾 EEPROM-Backed Config

All configurations (ESN, MQTT, WiFi, Network Mode, Heartbeat Interval) are saved to EEPROM and persist across reboots.

---

## 📂 File Structure

```bash
CLIHandler.h/.cpp       # Main CLI logic
EEPROMManager.h/.cpp    # Read/write EEPROM config
WiFiManager.h/.cpp      # WiFi-related logic
MQTTHandler.h/.cpp      # MQTT-related setup
```

---

## 🛠 Dependencies

- FreeRTOS (via ESP32)
- Arduino Core for ESP32
- EEPROM
- WiFi
- MQTT (if used)

---

## 📜 License

MIT License

---

## 🙋‍♂️ Contact

Maintained by **Charles Kim Kabiling**  
📧 [your.email@example.com]  
📍 Philippines

---
