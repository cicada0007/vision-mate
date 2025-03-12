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

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('walking_stick')

# GPIO Pins
TRIG = 23
ECHO = 24
LED_PIN = 17  # LED indicator for warnings
BUTTON_PIN = 27  # Emergency button

# Constants
FALL_THRESHOLD = 1.5  # g (acceleration due to gravity)
OBSTACLE_DISTANCE_THRESHOLD = 100  # cm
EMERGENCY_CONTACT = "+1234567890"  # Replace with actual emergency contact

# MPU6050 Constants
MPU6050_ADDR = 0x68
ACCEL_XOUT_H = 0x3B
PWR_MGMT_1 = 0x6B

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize text-to-speech engine
engine = pyttsx3.init()
engine.setProperty('rate', 150)  # Speed of speech

class MPU6050:
    def __init__(self, address=MPU6050_ADDR, bus=1):
        self.address = address
        self.bus = SMBus(bus)
        
        # Wake up the MPU6050
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0)
        logger.info("MPU6050 initialized")
        
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
    """Detect falls using accelerometer data"""
    if mpu is None:
        return False
    
    try:
        accel_data = mpu.get_accel_data()
        magnitude = np.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)
        
        # Check if the magnitude deviates significantly from 1g
        # A free-falling object experiences 0g, and impact causes high g
        if magnitude < 0.3 or magnitude > FALL_THRESHOLD:
            # Double check with a second reading
            time.sleep(0.1)
            accel_data = mpu.get_accel_data()
            magnitude = np.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)
            if magnitude < 0.3 or magnitude > FALL_THRESHOLD:
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
    """Send message via Bluetooth or speak it aloud"""
    logger.info(f"Message: {message}")
    
    if bluetooth_serial:
        try:
            bluetooth_serial.write(f"{message}\n".encode())
        except Exception as e:
            logger.error(f"Bluetooth communication error: {e}")
            # Fall back to speech if Bluetooth fails
            speak(message)
    else:
        # Use text-to-speech if Bluetooth isn't available
        speak(message)

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

def main_loop():
    """Main program loop"""
    last_obstacle_warning = 0
    last_gps_report = 0
    fall_detected_time = 0
    
    try:
        while True:
            current_time = time.time()
            
            # Check for falls
            if mpu and detect_fall():
                if current_time - fall_detected_time > 10:  # Limit alerts to once per 10 seconds
                    fall_detected_time = current_time
                    send_message("Fall detected! Are you okay?")
                    # Blink LED rapidly to indicate fall detection
                    for _ in range(5):
                        GPIO.output(LED_PIN, GPIO.HIGH)
                        time.sleep(0.2)
                        GPIO.output(LED_PIN, GPIO.LOW)
                        time.sleep(0.2)
                    
                    # Wait for a few seconds to see if button is pressed
                    time.sleep(5)
                    send_message("If you need help, press the emergency button.")
            
            # Get ultrasonic distance
            distance = get_distance()
            
            # Process camera frame if available
            obstacles = []
            if camera and camera.isOpened():
                ret, frame = camera.read()
                if ret:
                    # Process frame to detect obstacles
                    processed_frame, obstacles = process_frame(frame)
                    
                    # Display frame (remove in production)
                    cv2.imshow("Walking Stick Vision", processed_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
            
            # Check for obstacles from ultrasonic sensor
            if distance < OBSTACLE_DISTANCE_THRESHOLD:
                if current_time - last_obstacle_warning > 2:  # Limit warnings to once per 2 seconds
                    last_obstacle_warning = current_time
                    send_message(f"Obstacle ahead, {distance:.1f} centimeters!")
                    GPIO.output(LED_PIN, GPIO.HIGH)  # Turn on LED for warning
                    time.sleep(0.5)
                    GPIO.output(LED_PIN, GPIO.LOW)
            
            # Check for obstacles from camera
            if obstacles and current_time - last_obstacle_warning > 2:
                # Get closest obstacle
                closest = min(obstacles, key=lambda x: x['estimated_distance'])
                last_obstacle_warning = current_time
                
                # Generate directional warning
                send_message(f"{closest['class']} detected to your {closest['position']}!")
                GPIO.output(LED_PIN, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(LED_PIN, GPIO.LOW)
            
            # Periodic GPS reporting (every 5 minutes)
            if current_time - last_gps_report > 300:
                lat, lon = get_gps_location()
                if lat and lon:
                    last_gps_report = current_time
                    # Just log GPS, don't announce it unless requested
                    logger.info(f"Current location: Latitude {lat:.6f}, Longitude {lon:.6f}")
            
            # Sleep to reduce CPU usage
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        logger.info("Program terminated by user")
    finally:
        # Clean up
        if camera and camera.isOpened():
            camera.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        if bluetooth_serial:
            bluetooth_serial.close()
        logger.info("Resources released")

if __name__ == "__main__":
    # Startup announcement
    send_message("Walking stick system starting up. Please wait.")
    
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
    else:
        send_message("All systems operational. Walking stick ready.")
    
    # Start main loop
    main_loop()