# Voice-Controlled-Car
Python code to control a 4WD robot car on a Raspberry Pi with an Adafruit Motor HAT. Features three modes: autonomous obstacle avoidance, Bluetooth remote control, and voice command support.

# Python Controlled Raspberry Pi Robot Car

This project contains the Python code to control a 4-wheel robot car using a Raspberry Pi and an Adafruit Motor HAT. It is designed to be modular and easy to understand. The robot can operate in three different modes: autonomous obstacle avoidance, direct Bluetooth control, and voice command control.

## Features

- **Object-Oriented Design:** The code is encapsulated within a `RobotCar` class, making it clean, reusable, and easy to maintain.
- **Multiple Control Modes:**
    1.  **Obstacle Avoidance:** The robot uses an ultrasonic sensor mounted on a servo to detect obstacles, look for the clearest path, and navigate autonomously.
    2.  **Bluetooth Control:** The robot can be controlled manually by sending single-character commands ('F', 'B', 'L', 'R', 'S') from a mobile app or computer via a Bluetooth serial connection.
    3.  **Voice Control:** Similar to Bluetooth control, this mode interprets specific characters sent over serial as voice commands (e.g., '^' for forward). This requires a companion mobile app that converts speech to text/characters.
- **Graceful Shutdown:** Uses the `atexit` module to ensure all motors are stopped and GPIO pins are cleaned up properly when the script is terminated.

---

## Hardware Requirements

To build this robot, you will need the following components:

- **Raspberry Pi:** Any model with GPIO pins (e.g., Raspberry Pi 3, 4, or Zero W).
- **Adafruit DC & Stepper Motor HAT:** The controller board that sits on top of the Raspberry Pi to drive the motors and servo.
- **Robot Car Chassis:** A 4-wheel drive (4WD) chassis is recommended.
- **DC Motors:** 4 DC motors compatible with your chassis and the Motor HAT.
- **Wheels:** 4 wheels that fit your motors.
- **Ultrasonic Sensor (HC-SR04):** For distance measurement.
- **Micro Servo (SG90 or similar):** To pan the ultrasonic sensor.
- **Mounting Bracket:** For attaching the ultrasonic sensor to the servo.
- **Power Source:** A portable power bank or battery pack for the Raspberry Pi and a separate battery pack (e.g., 4xAA) for the Motor HAT.
- **Bluetooth Module (optional):** A serial Bluetooth module (like HC-05 or HC-06) connected to the Pi's UART pins for wireless control.
- **Jumper Wires:** To connect the components.

---

## Software & Library Requirements

This script is written for Python 3. You need to install the following libraries on your Raspberry Pi.

Open a terminal on your Raspberry Pi and run the following commands:

1.  **RPi.GPIO:** For controlling the ultrasonic sensor's GPIO pins.
    ```bash
    pip install RPi.GPIO
    ```

2.  **Adafruit CircuitPython MotorKit:** The modern library for controlling the Adafruit Motor HAT.
    ```bash
    pip install adafruit-circuitpython-motorkit
    ```
    *Note: The script uses the older `Adafruit_MotorHAT` library. The `adafruit-circuitpython-motorkit` is the recommended modern equivalent and should be compatible. If you face issues, you may need to find and install the legacy `Adafruit-Motor-HAT-Python-Library`.*

3.  **pyserial:** For communicating with the Bluetooth module.
    ```bash
    pip install pyserial
    ```

---

## Code Explanation

The entire logic is contained within a single Python script, `robot_car.py`.

### `RobotCar` Class

This class holds all the properties and methods for the robot.

-   **`__init__(self)`**: The constructor.
    -   Initializes the Adafruit Motor HAT.
    -   Creates objects for all 4 DC motors and the servo.
    -   Sets up the GPIO pins for the ultrasonic sensor (`TRIG_PIN` and `ECHO_PIN`).
    -   Attempts to initialize a serial connection for Bluetooth/voice control.
    -   Registers the `cleanup` method to run on script exit.

-   **`cleanup(self)`**: Stops all motors and releases GPIO resources to prevent issues on the next run.

### Control Modes

-   **`run_obstacle_avoidance(self)`**:
    -   Continuously measures the distance ahead.
    -   If an obstacle is detected within `OBSTACLE_DISTANCE`, it stops, backs up, looks left and right to measure distances, and then turns towards the direction with more free space.
    -   If the path is clear, it moves forward.

-   **`run_bluetooth_control(self)`**:
    -   Listens for incoming data on the serial port.
    -   Executes a move (Forward, Backward, Left, Right, Stop) based on the character received.

-   **`run_voice_control(self)`**:
    -   Functions identically to Bluetooth control but uses a different set of command characters (`^`, `-`, `<`, `>`, `*`).

### Helper Methods

-   **Motor Control (`move_forward`, `turn_left`, etc.)**: These methods send the appropriate commands (`FORWARD`, `BACKWARD`, `RELEASE`) to the motor objects.
-   **Sensor & Servo (`get_ultrasonic_distance`, `look_left`, etc.)**: These methods handle the logic for triggering the ultrasonic sensor, calculating distance, and pointing the servo.

---

## Setup & Usage

1.  **Hardware Assembly:**
    -   Assemble your robot chassis with the motors and wheels.
    -   Place the Adafruit Motor HAT on your Raspberry Pi's GPIO headers.
    -   Connect the four DC motors to the M1, M2, M3, and M4 ports on the Motor HAT.
    -   Connect the servo to the PWM 0 pins on the HAT.
    -   Connect the ultrasonic sensor's VCC to a 5V pin, GND to a GND pin, TRIG to GPIO 17, and ECHO to GPIO 18 on the Pi.
    -   (Optional) Connect your Bluetooth module's VCC, GND, TX, and RX pins to the Raspberry Pi's 5V, GND, GPIO 14 (TXD), and GPIO 15 (RXD) respectively.

2.  **Download the Code:**
    -   Clone this repository or download the `robot_car.py` script to your Raspberry Pi.

3.  **Select the Mode:**
    -   Open the `robot_car.py` script in a text editor.
    -   Navigate to the `loop(self)` method.
    -   By default, `self.run_obstacle_avoidance()` is active. To change modes, comment out the active line (by adding a `#` at the beginning) and uncomment the line for the mode you want to use.

    ```python
    def loop(self):
        # --- CHOOSE OPERATING MODE ---
        # Call the desired mode function here.
        self.run_obstacle_avoidance()
        # self.run_bluetooth_control()
        # self.run_voice_control()
    ```

4.  **Run the Script:**
    -   Open a terminal and navigate to the directory where you saved the script.
    -   Run the code with root privileges, which are often required for GPIO access:
    ```bash
    sudo python robot_car.py
    ```
    -   The robot will start operating in the mode you selected.
    -   To stop the script, press `Ctrl + C` in the terminal. The cleanup function will automatically run.
