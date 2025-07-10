# 🤖 Smart Arduino Robot Car (Modular C++ Project)

A collaborative robotics project built using Arduino, designed to operate in **three distinct modes**:

- ✅ **Manual (Bluetooth)** – Real-time movement via serial or Bluetooth commands  
- 🗣️ **Voice Control** – Navigate using predefined voice commands  
- 🧠 **Autonomous Mode** – Smart obstacle avoidance using ultrasonic sensing and servo scanning

---

## 🚀 Features

- 3 driving modes controlled via serial or Bluetooth (`'M'` key to switch)
- Clean, modular C++ implementation using `.cpp` and `.h` structure
- Real-time obstacle detection using **ultrasonic sensor (HC-SR04)**
- Dynamic decision-making with servo-based environmental scanning
- Supports Android Bluetooth control and voice interfaces

---

## 🧩 Modes & Controls

| Mode | Trigger | Description |
|------|---------|-------------|
| **Manual** | `'F'`, `'B'`, `'L'`, `'R'`, `'S'` | Forward, Backward, Left, Right, Stop |
| **Voice Control** | `'^'`, `'-'`, `'<'`, `'>'`, `'*'` | Similar to manual, with logic constraints |
| **Obstacle Avoidance** | Auto | Scans left/right and decides based on free space |

🔁 Press `'M'` to cycle between modes.

---

## 🛠️ Hardware Components

| Component | Description |
|----------|-------------|
| Arduino Uno | Microcontroller |
| Adafruit L293D Motor Shield | Motor control |
| 4 x DC Motors | For movement |
| HC-SR04 | Ultrasonic distance sensor |
| Servo Motor | For scanning |
| HC-05 (optional) | Bluetooth module |
| Battery Pack | Power supply |

---

## 🧠 Code Structure

This is a fully modular Arduino sketch:
- `main.ino` – Entry point, handles setup and loop
- All logic (motor control, sensors, modes) is modularized using standard C++ practices.

For simplicity, the version in this repo is a **combined `.ino` file**, ready to upload directly to your Arduino.

---

## 🛎️ How to Use

1. Clone/download this repository
2. Open the `.ino` file in the Arduino IDE
3. Connect your hardware as per pin definitions
4. Upload and test using Serial Monitor or a Bluetooth app (like Arduino Bluetooth Controller)
5. Use voice commands if connected with voice module or custom Android app

---

## 🧑‍🤝‍🧑 Team Collaboration

This project was built as a collaborative effort. Each team member contributed to:
- Code architecture and modularization
- Sensor integration and debugging
- Testing all three operation modes in field conditions
- Documentation and version control

---

## 📸 Demo (Optional)

> Add a short 30–60 second demo video link here, or embed a GIF or YouTube preview.

---

## 🧪 Future Enhancements

- Add real-time obstacle map using IR sensors
- Integrate PID for smoother navigation
- Control via Web or Mobile app (ESP32-based upgrade)
- Rechargeable power bank integration
- Speed tuning via serial commands (`+`, `-`)

---

## 📄 License

This project is open-source under the MIT License. Feel free to fork, modify, or contribute.

---

## 🙌 Let’s Connect

If you’re working on similar robotics, IoT, or embedded systems projects — happy to collaborate or help out.

> Drop a ⭐️ if you found this helpful!

