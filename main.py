# Robot Car Control in Python
#
#code for controlling
# a 4-wheel robot car. It is designed for a Raspberry Pi or similar
# single-board computer.
#
# It combines the logic from RobotCar.h, RobotCar.cpp, and the main .ino file
# into a single Python script.
#
# Required Libraries:
# - RPi.GPIO: For controlling the ultrasonic sensor and servo.
#   (pip install RPi.GPIO)
# - Adafruit_MotorHAT: For controlling the DC motors with an Adafruit Motor HAT.
#   (pip install adafruit-circuitpython-motorkit)
# - pyserial: For reading commands from a serial port (e.g., Bluetooth module).
#   (pip install pyserial)
#

import time
import atexit
import serial
import RPi.GPIO as GPIO
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

class RobotCar:
    """
    Encapsulates all functionality for the robot car, mirroring the C++ class.
    """

    # --- Pin and Configuration Definitions (using BCM numbering for Raspberry Pi) ---
    ECHO_PIN = 18  # GPIO 18 (Pin 12)
    TRIG_PIN = 17  # GPIO 17 (Pin 11)
    
    # Note: The servo is controlled via the MotorHAT's PWM driver, not a direct GPIO pin.

    # --- Constants ---
    MAX_SPEED = 170          # Default speed for the motors (0-255)
    SERVO_CENTER_POINT = 375 # Calibrated pulse length for servo center (adjust as needed)
    SERVO_LEFT_POINT = 550   # Pulse length for looking left
    SERVO_RIGHT_POINT = 200  # Pulse length for looking right
    OBSTACLE_DISTANCE = 20   # Distance in cm to trigger avoidance

    def __init__(self):
        """
        Constructor: Initializes hardware and settings.
        """
        print("Initializing Robot Car...")
        # --- Initialize Motor HAT ---
        # bottom hat is default address 0x60
        self.mh = Adafruit_MotorHAT(addr=0x60)

        # --- Get Motor Objects ---
        self.M1 = self.mh.getMotor(1)
        self.M2 = self.mh.getMotor(2)
        self.M3 = self.mh.getMotor(3)
        self.M4 = self.mh.getMotor(4)
        
        # --- Get Servo Object ---
        # The servo is connected to the PWM pin 0 on the Motor HAT
        self.ultrasonic_servo = self.mh.getPWM(0)
        self.ultrasonic_servo.setPWMFreq(60) # Set PWM frequency to 60Hz

        # --- Initialize GPIO for Ultrasonic Sensor ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIG_PIN, GPIO.OUT)
        GPIO.setup(self.ECHO_PIN, GPIO.IN)
        
        # --- Initialize Serial Port (for Bluetooth/Voice) ---
        self.serial_port = None
        try:
            # For Raspberry Pi 3/4, the serial port is typically '/dev/ttyS0'
            # For older models or USB-to-serial adapters, it might be '/dev/ttyAMA0' or '/dev/ttyUSB0'
            self.serial_port = serial.Serial('/dev/ttyS0', 9600, timeout=1)
            self.serial_port.flush()
            print("Serial port /dev/ttyS0 opened successfully.")
        except serial.SerialException as e:
            print(f"Warning: Could not open serial port. Bluetooth/Voice control will be disabled. Error: {e}")

        # --- Set Initial State ---
        self.set_speed(self.MAX_SPEED)
        self.center_servo()
        
        # --- Register Cleanup Function ---
        # This ensures motors are stopped and GPIO is cleaned up on exit.
        atexit.register(self.cleanup)

        print("Robot Initialized. Ready for commands.")

    def cleanup(self):
        """
        Stops all motors and cleans up GPIO resources.
        """
        print("\nCleaning up and shutting down.")
        self.stop_motors()
        GPIO.cleanup()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def set_speed(self, speed):
        """Sets the speed for all motors."""
        self.M1.setSpeed(speed)
        self.M2.setSpeed(speed)
        self.M3.setSpeed(speed)
        self.M4.setSpeed(speed)

    
    # --- Main Loop ---
    
    def loop(self):
        """
        Main loop to select and run the operating mode.
        """
        print("\nStarting main loop. Press Ctrl+C to exit.")
        print("Current mode: Obstacle Avoidance")
        while True:
            # --- CHOOSE OPERATING MODE ---
            # Call the desired mode function here.
            self.run_obstacle_avoidance()
            # self.run_bluetooth_control()
            # self.run_voice_control()

    
    # --- Control Modes ---
    
    def run_obstacle_avoidance(self):
        """Runs the robot in autonomous obstacle avoidance mode."""
        distance = self.get_ultrasonic_distance()
        print(f"Distance: {distance:.2f} cm")

        if distance <= self.OBSTACLE_DISTANCE:
            print("Obstacle detected!")
            self.stop_motors()
            self.move_backward()
            time.sleep(0.3)
            self.stop_motors()
            time.sleep(0.2)

            dist_left = self.look_left()
            print(f"  - Space to the left: {dist_left:.2f} cm")
            time.sleep(0.2)
            dist_right = self.look_right()
            print(f"  - Space to the right: {dist_right:.2f} cm")
            time.sleep(0.2)
            
            self.center_servo()
            time.sleep(0.3)

            if dist_left > dist_right:
                print("-> Turning Left")
                self.turn_left()
                time.sleep(0.5)
            else:
                print("-> Turning Right")
                self.turn_right()
                time.sleep(0.5)
            self.stop_motors()
        else:
            print("Path clear. Moving forward.")
            self.move_forward()
        time.sleep(0.1) # Small delay to prevent spamming

    def run_bluetooth_control(self):
        """Listens for single-character commands via Serial."""
        if self.serial_port and self.serial_port.in_waiting > 0:
            command = self.serial_port.read().decode('utf-8').strip()
            print(f"Bluetooth Command Received: {command}")
            if command == 'F': self.move_forward()
            elif command == 'B': self.move_backward()
            elif command == 'L': self.turn_left()
            elif command == 'R': self.turn_right()
            elif command == 'S': self.stop_motors()

    def run_voice_control(self):
        """Listens for specific characters representing voice commands."""
        if self.serial_port and self.serial_port.in_waiting > 0:
            command = self.serial_port.read().decode('utf-8').strip()
            print(f"Voice Command Received: {command}")
            if command == '^': self.move_forward()
            elif command == '-': self.move_backward()
            elif command == '<': self.turn_left(); time.sleep(0.5); self.stop_motors()
            elif command == '>': self.turn_right(); time.sleep(0.5); self.stop_motors()
            elif command == '*': self.stop_motors()

    
    # --- Sensor and Servo Functions ---
    
    def get_ultrasonic_distance(self):
        """Measures distance using the ultrasonic sensor."""
        GPIO.output(self.TRIG_PIN, False)
        time.sleep(0.01) # Wait for sensor to settle

        GPIO.output(self.TRIG_PIN, True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG_PIN, False)

        pulse_start_time = time.time()
        pulse_end_time = time.time()

        while GPIO.input(self.ECHO_PIN) == 0:
            pulse_start_time = time.time()

        while GPIO.input(self.ECHO_PIN) == 1:
            pulse_end_time = time.time()

        pulse_duration = pulse_end_time - pulse_start_time
        distance = pulse_duration * 17150  # Speed of sound (34300 cm/s) / 2
        return round(distance, 2)

    def center_servo(self):
        self.ultrasonic_servo.setPWM(0, 0, self.SERVO_CENTER_POINT)

    def look_left(self):
        self.ultrasonic_servo.setPWM(0, 0, self.SERVO_LEFT_POINT)
        time.sleep(0.5)
        return self.get_ultrasonic_distance()

    def look_right(self):
        self.ultrasonic_servo.setPWM(0, 0, self.SERVO_RIGHT_POINT)
        time.sleep(0.5)
        return self.get_ultrasonic_distance()

    
    # --- Motor Control Functions ---
   
    def move_forward(self):
        self.M1.run(Adafruit_MotorHAT.FORWARD)
        self.M2.run(Adafruit_MotorHAT.FORWARD)
        self.M3.run(Adafruit_MotorHAT.FORWARD)
        self.M4.run(Adafruit_MotorHAT.FORWARD)

    def move_backward(self):
        self.M1.run(Adafruit_MotorHAT.BACKWARD)
        self.M2.run(Adafruit_MotorHAT.BACKWARD)
        self.M3.run(Adafruit_MotorHAT.BACKWARD)
        self.M4.run(Adafruit_MotorHAT.BACKWARD)

    def turn_right(self):
        self.M1.run(Adafruit_MotorHAT.FORWARD)
        self.M2.run(Adafruit_MotorHAT.FORWARD)
        self.M3.run(Adafruit_MotorHAT.BACKWARD)
        self.M4.run(Adafruit_MotorHAT.BACKWARD)

    def turn_left(self):
        self.M1.run(Adafruit_MotorHAT.BACKWARD)
        self.M2.run(Adafruit_MotorHAT.BACKWARD)
        self.M3.run(Adafruit_MotorHAT.FORWARD)
        self.M4.run(Adafruit_MotorHAT.FORWARD)

    def stop_motors(self):
        self.M1.run(Adafruit_MotorHAT.RELEASE)
        self.M2.run(Adafruit_MotorHAT.RELEASE)
        self.M3.run(Adafruit_MotorHAT.RELEASE)
        self.M4.run(Adafruit_MotorHAT.RELEASE)

# Main Execution Block
if __name__ == '__main__':
    try:
        # Create a single instance of our robot car
        my_car = RobotCar()
        # Start the main loop
        my_car.loop()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # The atexit handler will automatically call cleanup(),
        # but we can also explicitly clean up GPIO here.
        GPIO.cleanup()