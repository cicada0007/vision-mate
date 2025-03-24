#!/usr/bin/env python3

import RPi.GPIO as GPIO
import cv2
import time
import serial
import gpsd
import threading
import numpy as np
import os
from smbus2 import SMBus
import pyttsx3
import logging
from PIL import Image
import sounddevice as sd
import subprocess
from threading import Lock

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('walking_stick')

# GPIO Pins
TRIG = 23  # Ultrasonic sensor trigger
ECHO = 24  # Ultrasonic sensor echo
LED_PIN = 17  # LED indicator for warnings
BUTTON_PIN = 27  # Emergency button
VIBRATION_MOTOR_PIN = 18  # Vibration motor for tactile feedback
BUZZER_PIN = 22  # Buzzer for audio alerts
LED_RED_PIN = 5  # Red LED for danger alerts
LED_GREEN_PIN = 6  # Green LED for status indication
LED_BLUE_PIN = 13  # Blue LED for Bluetooth connection status

# Constants
FALL_THRESHOLD = 1.5  # g (acceleration due to gravity)
OBSTACLE_DISTANCE_THRESHOLD = 100  # cm
EMERGENCY_CONTACT = "+1234567890"  # Replace with actual emergency contact

# MPU6050 Constants and I2C pins
MPU6050_ADDR = 0x68
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C

# I2C Pins for MPU6050 (These are fixed for Raspberry Pi)
I2C_SCL = 3  # GPIO3 (SCL)
I2C_SDA = 2  # GPIO2 (SDA)

# MPU6050 Scale Configuration
GYRO_SCALE_250 = 0x00   # ±250 degrees/second
GYRO_SCALE_500 = 0x08   # ±500 degrees/second
GYRO_SCALE_1000 = 0x10  # ±1000 degrees/second
GYRO_SCALE_2000 = 0x18  # ±2000 degrees/second

ACCEL_SCALE_2G = 0x00   # ±2g
ACCEL_SCALE_4G = 0x08   # ±4g
ACCEL_SCALE_8G = 0x10   # ±8g
ACCEL_SCALE_16G = 0x18  # ±16g

# MPU6050 Internal Registers
MPU6050_WHO_AM_I = 0x75
MPU6050_SMPLRT_DIV = 0x19
MPU6050_CONFIG = 0x1A
MPU6050_INT_ENABLE = 0x38
MPU6050_INT_STATUS = 0x3A

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(VIBRATION_MOTOR_PIN, GPIO.OUT)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(LED_RED_PIN, GPIO.OUT)
GPIO.setup(LED_GREEN_PIN, GPIO.OUT)
GPIO.setup(LED_BLUE_PIN, GPIO.OUT)

# Initialize text-to-speech engine
engine = pyttsx3.init()
engine.setProperty('rate', 150)  # Speed of speech

# Initialize output pins
GPIO.output(VIBRATION_MOTOR_PIN, GPIO.LOW)
GPIO.output(BUZZER_PIN, GPIO.LOW)
GPIO.output(LED_RED_PIN, GPIO.LOW)
GPIO.output(LED_GREEN_PIN, GPIO.LOW)
GPIO.output(LED_BLUE_PIN, GPIO.LOW)

# Add audio device management
audio_lock = Lock()
audio_device = None
AUDIO_SAMPLE_RATE = 44100

class MPU6050:
    def __init__(self, address=MPU6050_ADDR, bus=1):
        self.address = address
        self.bus = SMBus(bus)
        
        # Wake up the MPU6050
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0)
        
        # Verify device ID
        who_am_i = self.bus.read_byte_data(self.address, MPU6050_WHO_AM_I)
        if who_am_i != 0x68:
            raise RuntimeError(f"MPU6050 not found! Device ID: {who_am_i}")
        
        # Configure sample rate (1kHz)
        self.bus.write_byte_data(self.address, MPU6050_SMPLRT_DIV, 0x07)
        
        # Configure gyroscope (±250°/s range)
        self.bus.write_byte_data(self.address, GYRO_CONFIG, GYRO_SCALE_250)
        
        # Configure accelerometer (±2g range)
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, ACCEL_SCALE_2G)
        
        # Enable data ready interrupt
        self.bus.write_byte_data(self.address, MPU6050_INT_ENABLE, 0x01)
        
        logger.info("MPU6050 initialized with gyroscope")
        
    def read_raw_data(self, addr):
        # Read the high and low 8-bit values
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        
        # Combine the high and low values
        value = (high << 8) | low
        
        # Convert to signed value
        if value > 32767:
            value -= 65536
            
        return value
    
    def get_accel_data(self):
        # Read accelerometer data
        accel_x = self.read_raw_data(ACCEL_XOUT_H) / 16384.0  # Convert to g
        accel_y = self.read_raw_data(ACCEL_XOUT_H + 2) / 16384.0
        accel_z = self.read_raw_data(ACCEL_XOUT_H + 4) / 16384.0
        
        return {'x': accel_x, 'y': accel_y, 'z': accel_z}
    
    def get_gyro_data(self):
        """Read gyroscope data
        Returns:
            dict: Contains angular velocities in degrees/s for x, y, z axes
        """
        try:
            # Read raw gyroscope data
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_XOUT_H + 2)
            gyro_z = self.read_raw_data(GYRO_XOUT_H + 4)
            
            # Convert to degrees/s (±250°/s range -> ±32768)
            # Sensitivity = 131 LSB/(degrees/s)
            gyro_x = gyro_x / 131.0
            gyro_y = gyro_y / 131.0
            gyro_z = gyro_z / 131.0
            
            return {'x': gyro_x, 'y': gyro_y, 'z': gyro_z}
        except Exception as e:
            logger.error(f"Gyroscope read error: {e}")
            return {'x': 0, 'y': 0, 'z': 0}
    
    def get_motion_data(self):
        """Get combined accelerometer and gyroscope data
        Returns:
            dict: Contains both acceleration and angular velocity data
        """
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()
        return {
            'accel': accel,
            'gyro': gyro
        }

# Initialize MPU6050
try:
    mpu = MPU6050()
    logger.info("MPU6050 successfully initialized")
except Exception as e:
    logger.error(f"Error initializing MPU6050: {e}")
    mpu = None

# Initialize GPS Module
try:
    gpsd.connect()
    logger.info("GPS connected")
except Exception as e:
    logger.error(f"GPS connection error: {e}")

# Try to initialize Bluetooth
bluetooth_serial = None
try:
    # Use rfcomm0 if you've already paired and connected your Bluetooth device
    bluetooth_serial = serial.Serial('/dev/rfcomm0', baudrate=9600, timeout=1)
    logger.info("Bluetooth connected")
except Exception as e:
    logger.error(f"Bluetooth connection error: {e}")
    # Fall back to console output if Bluetooth isn't available
    logger.info("Will use text-to-speech instead of Bluetooth")

# Initialize YOLO model
model = None
class_names = []

try:
    # Check if we have YOLO tiny weights and config files
    if os.path.exists('yolov4-tiny.weights') and os.path.exists('yolov4-tiny.cfg'):
        # Load YOLO using OpenCV's DNN module instead of TensorFlow
        net = cv2.dnn.readNet('yolov4-tiny.weights', 'yolov4-tiny.cfg')
        
        # Specify backends - prefer CUDA if available, fall back to CPU
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA if cv2.cuda.getCudaEnabledDeviceCount() > 0 else cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA if cv2.cuda.getCudaEnabledDeviceCount() > 0 else cv2.dnn.DNN_TARGET_CPU)
        
        model = cv2.dnn_DetectionModel(net)
        model.setInputParams(size=(416, 416), scale=1/255, swapRB=True)
        
        # Load YOLO Class Names
        if os.path.exists("coco.names"):
            with open("coco.names", "r") as f:
                class_names = f.read().splitlines()
        else:
            # Common COCO class names if file is missing
            class_names = ["person", "bicycle", "car", "motorcycle", "bus", "truck", "fire hydrant", 
                           "stop sign", "bench", "bird", "cat", "dog", "backpack", "umbrella"]
            logger.warning("coco.names file not found. Using default class list.")
        
        logger.info("YOLO model loaded")
    else:
        logger.error("YOLO model files not found!")
except Exception as e:
    logger.error(f"Error loading YOLO model: {e}")

# Initialize camera
camera = None
try:
    camera = cv2.VideoCapture(0)
    # Set lower resolution for better performance
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    if not camera.isOpened():
        logger.error("Failed to open camera")
    else:
        logger.info("Camera initialized")
except Exception as e:
    logger.error(f"Camera initialization error: {e}")

def speak(message):
    """Convert text to speech and play it"""
    try:
        engine.say(message)
        engine.runAndWait()
    except Exception as e:
        logger.error(f"Text-to-speech error: {e}")

def get_distance():
    """Get distance using Ultrasonic Sensor"""
    try:
        # Clear the trigger pin
        GPIO.output(TRIG, False)
        time.sleep(0.2)  # Let sensor settle
        
        # Send 10us pulse to trigger
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)
        
        start_time = time.time()
        stop_time = time.time()
        
        # Save start time
        while GPIO.input(ECHO) == 0:
            start_time = time.time()
            # Timeout to prevent infinite loop
            if time.time() - stop_time > 0.1:
                return float('inf')
        
        # Save time of arrival
        while GPIO.input(ECHO) == 1:
            stop_time = time.time()
            # Timeout to prevent infinite loop
            if stop_time - start_time > 0.1:
                return float('inf')
        
        # Calculate distance
        time_elapsed = stop_time - start_time
        distance = (time_elapsed * 34300) / 2  # Speed of sound = 343 m/s
        
        return distance
    except Exception as e:
        logger.error(f"Distance measurement error: {e}")
        return float('inf')  # Return infinity if there's an error

def detect_fall():
    """Detect falls using accelerometer and gyroscope data"""
    if mpu is None:
        return False
    
    try:
        motion_data = mpu.get_motion_data()
        accel_data = motion_data['accel']
        gyro_data = motion_data['gyro']
        
        # Calculate acceleration magnitude
        accel_magnitude = np.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)
        
        # Calculate angular velocity magnitude
        gyro_magnitude = np.sqrt(gyro_data['x']**2 + gyro_data['y']**2 + gyro_data['z']**2)
        
        # Enhanced fall detection using both accelerometer and gyroscope
        if (accel_magnitude < 0.3 or accel_magnitude > FALL_THRESHOLD) and gyro_magnitude > 100:
            # Double check with a second reading after a short delay
            time.sleep(0.1)
            motion_data = mpu.get_motion_data()
            accel_data = motion_data['accel']
            
            # Verify impact after potential fall
            accel_magnitude = np.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)
            if accel_magnitude > FALL_THRESHOLD:
                return True
        return False
    except Exception as e:
        logger.error(f"Fall detection error: {e}")
        return False

def get_gps_location():
    """Retrieve current GPS coordinates"""
    try:
        packet = gpsd.get_current()
        if packet.mode >= 2:  # 2D Fix or better
            return packet.lat, packet.lon
        return None, None
    except Exception as e:
        logger.error(f"GPS error: {e}")
        return None, None

def send_message(message):
    """Send message through wireless headphones or fallback to system audio"""
    global audio_device
    logger.info(f"Message: {message}")
    
    with audio_lock:
        try:
            # Try to speak through wireless device if available
            if audio_device is not None:
                try:
                    engine.setProperty('device', audio_device)
                    engine.say(message)
                    engine.runAndWait()
                    return
                except Exception as e:
                    logger.error(f"Error using wireless audio device: {e}")
                    audio_device = None  # Reset device on error
            
            # Fallback to default audio output
            engine.setProperty('device', None)  # Use system default
            engine.say(message)
            engine.runAndWait()
        except Exception as e:
            logger.error(f"Text-to-speech error: {e}")

def process_frame(frame):
    """Process video frame to detect obstacles"""
    if model is None or not class_names:
        return frame
    
    try:
        # Detect objects
        classes, scores, boxes = model.detect(frame, confThreshold=0.5, nmsThreshold=0.4)
        
        obstacles = []
        
        # Draw detections
        for (classid, score, box) in zip(classes, scores, boxes):
            if classid < len(class_names):  # Ensure class ID is valid
                label = f"{class_names[classid]}: {score:.2f}"
                
                # Calculate position in frame (useful for directional warnings)
                x, y, w, h = box
                center_x = x + w // 2
                frame_center_x = frame.shape[1] // 2
                position = "left" if center_x < frame_center_x else "right"
                
                # Calculate approximate distance based on object size
                # This is a rough estimate; ultrasonic sensor provides more accurate distance
                distance_estimate = 500 * (100 / w)  # Rough formula
                
                # Add to obstacles list
                obstacles.append({
                    'class': class_names[classid],
                    'position': position,
                    'estimated_distance': distance_estimate,
                    'box': box
                })
                
                # Draw rectangle and label
                cv2.rectangle(frame, box, (0, 255, 0), 2)
                cv2.putText(frame, label, (box[0], box[1] - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Return processed frame and obstacles
        return frame, obstacles
    except Exception as e:
        logger.error(f"Frame processing error: {e}")
        return frame, []

def emergency_button_callback(channel):
    """Handle emergency button press"""
    send_message("Emergency button pressed. Sending alert.")
    # Get GPS coordinates
    lat, lon = get_gps_location()
    location_str = f"at latitude {lat:.6f}, longitude {lon:.6f}" if lat and lon else "unknown location"
    send_message(f"Emergency alert sent with {location_str}")
    # In a real implementation, you would send SMS/call here
    logger.info(f"EMERGENCY ALERT: {location_str}")

# Set up emergency button interrupt
GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=emergency_button_callback, bouncetime=2000)

def vibrate(duration=0.5, intensity=100):
    """Activate vibration motor for tactile feedback
    
    Args:
        duration (float): Duration in seconds
        intensity (int): PWM intensity (0-100)
    """
    try:
        # Create PWM object
        pwm = GPIO.PWM(VIBRATION_MOTOR_PIN, 100)  # 100 Hz frequency
        pwm.start(intensity)
        time.sleep(duration)
        pwm.stop()
    except Exception as e:
        logger.error(f"Vibration motor error: {e}")

def buzz(duration=0.5, pattern=None):
    """Activate buzzer for audio alerts
    
    Args:
        duration (float): Duration in seconds
        pattern (list): List of (on_time, off_time) tuples for patterned buzz
    """
    try:
        if pattern:
            end_time = time.time() + duration
            while time.time() < end_time:
                for on_time, off_time in pattern:
                    GPIO.output(BUZZER_PIN, GPIO.HIGH)
                    time.sleep(on_time)
                    GPIO.output(BUZZER_PIN, GPIO.LOW)
                    time.sleep(off_time)
                    if time.time() >= end_time:
                        break
        else:
            GPIO.output(BUZZER_PIN, GPIO.HIGH)
            time.sleep(duration)
            GPIO.output(BUZZER_PIN, GPIO.LOW)
    except Exception as e:
        logger.error(f"Buzzer error: {e}")

def set_led_color(red=False, green=False, blue=False):
    """Set the color of the RGB LED
    
    Args:
        red (bool): Whether to turn on red LED
        green (bool): Whether to turn on green LED
        blue (bool): Whether to turn on blue LED
    """
    try:
        GPIO.output(LED_RED_PIN, GPIO.HIGH if red else GPIO.LOW)
        GPIO.output(LED_GREEN_PIN, GPIO.HIGH if green else GPIO.LOW)
        GPIO.output(LED_BLUE_PIN, GPIO.HIGH if blue else GPIO.LOW)
    except Exception as e:
        logger.error(f"LED control error: {e}")

def alert_pattern(type='warning', duration=2.0):
    """Execute an alert pattern using LEDs, vibration, and buzzer
    
    Args:
        type (str): Type of alert - 'warning', 'danger', or 'info'
        duration (float): Duration in seconds
    """
    try:
        if type == 'danger':
            # Red LED + strong vibration + rapid beeping
            set_led_color(red=True)
            threading.Thread(target=vibrate, args=(duration, 100)).start()
            buzz(duration, pattern=[(0.1, 0.1)] * 10)  # Rapid beeping
        elif type == 'warning':
            # Yellow LED (red + green) + medium vibration + intermittent beep
            set_led_color(red=True, green=True)
            threading.Thread(target=vibrate, args=(duration, 70)).start()
            buzz(duration, pattern=[(0.2, 0.3)] * 5)  # Intermittent beeping
        elif type == 'info':
            # Blue LED + gentle vibration + single beep
            set_led_color(blue=True)
            threading.Thread(target=vibrate, args=(duration, 40)).start()
            buzz(0.5)  # Single beep
        
        # Reset LEDs after alert
        time.sleep(duration)
        set_led_color()
    except Exception as e:
        logger.error(f"Alert pattern error: {e}")

def update_bluetooth_status(connected=False):
    """Update the Bluetooth connection status LED
    
    Args:
        connected (bool): Whether Bluetooth is connected
    """
    try:
        if connected:
            GPIO.output(LED_BLUE_PIN, GPIO.HIGH)
        else:
            GPIO.output(LED_BLUE_PIN, GPIO.LOW)
    except Exception as e:
        logger.error(f"Bluetooth status LED error: {e}")

def get_wireless_audio_devices():
    """Get list of available wireless audio output devices"""
    try:
        devices = sd.query_devices()
        wireless_devices = []
        for i, device in enumerate(devices):
            if device['max_output_channels'] > 0 and ('bluetooth' in device['name'].lower() or 'wireless' in device['name'].lower()):
                wireless_devices.append((i, device['name']))
        return wireless_devices
    except Exception as e:
        logger.error(f"Error getting audio devices: {e}")
        return []

def connect_wireless_audio():
    """Try to connect to a wireless audio device"""
    global audio_device
    
    try:
        wireless_devices = get_wireless_audio_devices()
        
        if wireless_devices:
            # Use the first available wireless device
            device_id, device_name = wireless_devices[0]
            audio_device = device_id
            logger.info(f"Connected to wireless audio device: {device_name}")
            return True
        else:
            logger.warning("No wireless audio devices found")
            return False
    except Exception as e:
        logger.error(f"Error connecting to wireless audio: {e}")
        return False

# Add hardware detection function
def check_hardware():
    """Check hardware connections and report status"""
    hardware_status = {
        "mpu6050": {
            "status": False,
            "message": "Not detected"
        },
        "gpio": {
            "status": False,
            "message": "Not configured"
        },
        "camera": {
            "status": False,
            "message": "Not connected"
        },
        "audio": {
            "status": False,
            "message": "No audio device"
        },
        "gps": {
            "status": False,
            "message": "No fix"
        }
    }
    
    # Test MPU6050
    try:
        bus = SMBus(1)
        who_am_i = bus.read_byte_data(MPU6050_ADDR, MPU6050_WHO_AM_I)
        if who_am_i == 0x68:
            hardware_status["mpu6050"]["status"] = True
            hardware_status["mpu6050"]["message"] = "Connected"
    except Exception as e:
        hardware_status["mpu6050"]["message"] = f"Error: {str(e)}"
    
    # Test GPIO
    try:
        test_pins = [TRIG, ECHO, LED_PIN, BUTTON_PIN, VIBRATION_MOTOR_PIN, 
                     BUZZER_PIN, LED_RED_PIN, LED_GREEN_PIN, LED_BLUE_PIN]
        for pin in test_pins:
            GPIO.setup(pin, GPIO.OUT)
        hardware_status["gpio"]["status"] = True
        hardware_status["gpio"]["message"] = "Configured"
    except Exception as e:
        hardware_status["gpio"]["message"] = f"Error: {str(e)}"
    
    # Test Camera
    try:
        if camera and camera.isOpened():
            hardware_status["camera"]["status"] = True
            hardware_status["camera"]["message"] = "Connected"
        else:
            hardware_status["camera"]["message"] = "Not available"
    except Exception as e:
        hardware_status["camera"]["message"] = f"Error: {str(e)}"
    
    # Test Audio
    try:
        if audio_device is not None:
            hardware_status["audio"]["status"] = True
            hardware_status["audio"]["message"] = "Connected"
        else:
            wireless_devices = get_wireless_audio_devices()
            if wireless_devices:
                hardware_status["audio"]["status"] = True
                hardware_status["audio"]["message"] = "Available"
            else:
                hardware_status["audio"]["message"] = "No wireless devices found"
    except Exception as e:
        hardware_status["audio"]["message"] = f"Error: {str(e)}"
    
    # Test GPS
    try:
        packet = gpsd.get_current()
        if packet.mode >= 2:
            hardware_status["gps"]["status"] = True
            hardware_status["gps"]["message"] = f"Fix acquired ({packet.mode}D)"
        else:
            hardware_status["gps"]["message"] = "No GPS fix"
    except Exception as e:
        hardware_status["gps"]["message"] = f"Error: {str(e)}"
    
    return hardware_status

# Add get_hardware_status function
def get_hardware_status():
    """Get current hardware status for web interface"""
    try:
        status = check_hardware()
        # Add version information
        status["system_info"] = {
            "python_version": sys.version.split()[0],
            "opencv_version": cv2.__version__,
            "user": os.getenv('USER', 'unknown')
        }
        return status
    except Exception as e:
        logger.error(f"Error getting hardware status: {e}")
        return None

# Modify startup_sequence to use hardware check
def startup_sequence():
    """Execute startup sequence to test all components"""
    try:
        logger.info("Starting hardware detection sequence...")
        
        # Check all hardware components
        hw_status = check_hardware()
        
        # Report results
        for component, status in hw_status.items():
            if status:
                logger.info(f"{component.upper()}: CONNECTED ✓")
            else:
                logger.error(f"{component.UPPER()}: NOT DETECTED ✗")
        
        # Add detailed I2C debugging
        if not hw_status["mpu6050"]:
            try:
                logger.info("Scanning I2C bus...")
                os.system('i2cdetect -y 1')
                logger.info("Checking I2C permissions...")
                os.system('ls -l /dev/i2c*')
            except Exception as e:
                logger.error(f"I2C debug failed: {str(e)}")
        
        # Continue with rest of startup sequence
        # Try to connect to wireless audio device
        if connect_wireless_audio():
            logger.info("Wireless audio device connected")
        else:
            logger.warning("Using system default audio output")
        
        # Test all LEDs
        logger.info("Testing LEDs...")
        set_led_color(red=True)
        time.sleep(0.5)
        set_led_color(green=True)
        time.sleep(0.5)
        set_led_color(blue=True)
        time.sleep(0.5)
        set_led_color()
        
        # Test vibration motor
        logger.info("Testing vibration motor...")
        vibrate(0.5, 50)
        
        # Test buzzer
        logger.info("Testing buzzer...")
        buzz(0.3)
        time.sleep(0.2)
        buzz(0.3)
        
        logger.info("Startup sequence completed")
        return True
    except Exception as e:
        logger.error(f"Startup sequence error: {str(e)}")
        return False

# Modify main to include more detailed error reporting
if __name__ == "__main__":
    try:
        logger.info("=== Vision Mate Starting ===")
        logger.info(f"Python version: {sys.version}")
        logger.info(f"OpenCV version: {cv2.__version__}")
        logger.info(f"Running as user: {os.getenv('USER')}")
        logger.info("Checking hardware permissions...")
        os.system('groups')
        
        # Start the system
        send_message("Walking stick system starting up. Please wait.")
        startup_result = startup_sequence()
        
        # Check if all components initialized properly
        missing_components = []
        if mpu is None:
            missing_components.append("accelerometer")
        if camera is None or not camera.isOpened():
            missing_components.append("camera")
        if model is None:
            missing_components.append("obstacle detection model")
        if bluetooth_serial is None:
            missing_components.append("Bluetooth")
        
        if missing_components:
            send_message(f"Warning: The following components are not working: {', '.join(missing_components)}. Limited functionality available.")
            # Use warning alert pattern
            alert_pattern('warning', 2.0)
        else:
            send_message("All systems operational. Walking stick ready.")
            # Use info alert pattern
            alert_pattern('info', 2.0)
        
        # Start main loop
        main_loop()
        
    except Exception as e:
        logger.error(f"Critical error during startup: {str(e)}")
        send_message("System encountered critical error. Please check logs.")

def main_loop():
    """Main program loop"""
    last_hardware_check = 0
    hardware_check_interval = 5  # Check hardware every 5 seconds
    
    while True:
        try:
            current_time = time.time()
            
            # Periodic hardware status check
            if current_time - last_hardware_check > hardware_check_interval:
                hardware_status = get_hardware_status()
                device_status["hardware"] = hardware_status
                last_hardware_check = current_time
            
            # Rest of your main loop code...
            
            time.sleep(0.1)  # Prevent CPU overuse
            
        except Exception as e:
            logger.error(f"Error in main loop: {e}")
            time.sleep(1)

@socketio.on('request_hardware_status')
def handle_hardware_status_request():
    """Handle hardware status request from web client"""
    try:
        status = get_hardware_status()
        emit('hardware_status', status)
    except Exception as e:
        logger.error(f"Error sending hardware status: {e}")
        emit('hardware_status', {"error": str(e)})