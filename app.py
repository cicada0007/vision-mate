from flask import Flask, render_template, request, jsonify, Response
from flask_socketio import SocketIO, emit
import cv2
import threading
import time
import json
import base64
import logging
import os
import importlib.util
import sys
from threading import Thread, Lock
import random
import numpy as np
import math
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('walking_stick_web')

# Initialize Flask app
app = Flask(__name__)
app.config['SECRET_KEY'] = 'walking_stick_secret_key'
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins="*")

# Global variables
walking_stick = None
camera_feed_active = False
camera_thread = None
thread_lock = Lock()
thread = None
camera = None
last_frame = None
client_count = 0

# Import walking_stick module
try:
    # Add current directory to path
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    
    # Import walking_stick module
    spec = importlib.util.spec_from_file_location("walking_stick", "walking_stick.py")
    walking_stick_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(walking_stick_module)
    
    logger.info("Walking stick module imported successfully")
except Exception as e:
    logger.error(f"Failed to import walking stick module: {e}")
    walking_stick_module = None

# Device status
device_status = {
    "connected": False,
    "distance": 0,
    "fall_detected": False,
    "gps_location": {"lat": None, "lon": None},
    "accelerometer": {"x": 0, "y": 0, "z": 0},
    "obstacles": [],
    "components_status": {
        "camera": False,
        "accelerometer": False,
        "gps": False,
        "bluetooth": False,
        "model": False
    },
    "gpio_status": {
        "led_red": False,
        "led_green": False,
        "led_blue": False,
        "vibration_motor": False,
        "buzzer": False
    }
}

@app.route('/')
def index():
    """Render the main page of the web application"""
    return render_template('index.html')

@app.route('/api/status')
def get_status():
    """Return the current status of the walking stick"""
    return jsonify(device_status)

@app.route('/api/speak', methods=['POST'])
def speak_message():
    """Speak a message through the walking stick"""
    data = request.json
    message = data.get('message', '')
    
    if walking_stick_module and hasattr(walking_stick_module, 'send_message'):
        try:
            walking_stick_module.send_message(message)
            return jsonify({"status": "success", "message": f"Spoke: {message}"})
        except Exception as e:
            logger.error(f"Failed to speak message: {e}")
            return jsonify({"status": "error", "message": str(e)})
    else:
        return jsonify({"status": "error", "message": "Walking stick module not available"})

@app.route('/api/emergency', methods=['POST'])
def trigger_emergency():
    """Trigger the emergency functionality"""
    if walking_stick_module and hasattr(walking_stick_module, 'emergency_button_callback'):
        try:
            # Call emergency function from walking stick
            walking_stick_module.emergency_button_callback(None)
            return jsonify({"status": "success", "message": "Emergency alert triggered"})
        except Exception as e:
            logger.error(f"Failed to trigger emergency: {e}")
            return jsonify({"status": "error", "message": str(e)})
    else:
        return jsonify({"status": "error", "message": "Walking stick module not available"})

@app.route('/api/vibrate', methods=['POST'])
def trigger_vibration():
    """Trigger vibration motor"""
    data = request.json
    duration = data.get('duration', 0.5)
    intensity = data.get('intensity', 100)
    
    if walking_stick_module and hasattr(walking_stick_module, 'vibrate'):
        try:
            walking_stick_module.vibrate(duration, intensity)
            return jsonify({"status": "success", "message": f"Vibration activated: {duration}s at {intensity}% intensity"})
        except Exception as e:
            logger.error(f"Failed to trigger vibration: {e}")
            return jsonify({"status": "error", "message": str(e)})
    else:
        return jsonify({"status": "error", "message": "Vibration function not available"})

@app.route('/api/buzz', methods=['POST'])
def trigger_buzzer():
    """Trigger buzzer"""
    data = request.json
    duration = data.get('duration', 0.5)
    pattern = data.get('pattern', None)
    
    if walking_stick_module and hasattr(walking_stick_module, 'buzz'):
        try:
            walking_stick_module.buzz(duration, pattern)
            return jsonify({"status": "success", "message": f"Buzzer activated for {duration}s"})
        except Exception as e:
            logger.error(f"Failed to trigger buzzer: {e}")
            return jsonify({"status": "error", "message": str(e)})
    else:
        return jsonify({"status": "error", "message": "Buzzer function not available"})

@app.route('/api/led', methods=['POST'])
def control_led():
    """Control LEDs"""
    data = request.json
    red = data.get('red', False)
    green = data.get('green', False)
    blue = data.get('blue', False)
    
    if walking_stick_module and hasattr(walking_stick_module, 'set_led_color'):
        try:
            walking_stick_module.set_led_color(red, green, blue)
            return jsonify({"status": "success", "message": f"LED set to R:{red} G:{green} B:{blue}"})
        except Exception as e:
            logger.error(f"Failed to control LED: {e}")
            return jsonify({"status": "error", "message": str(e)})
    else:
        return jsonify({"status": "error", "message": "LED function not available"})

@app.route('/api/alert', methods=['POST'])
def trigger_alert():
    """Trigger alert pattern"""
    data = request.json
    alert_type = data.get('type', 'info')
    duration = data.get('duration', 2.0)
    
    if walking_stick_module and hasattr(walking_stick_module, 'alert_pattern'):
        try:
            walking_stick_module.alert_pattern(alert_type, duration)
            return jsonify({"status": "success", "message": f"Alert pattern activated: {alert_type} for {duration}s"})
        except Exception as e:
            logger.error(f"Failed to trigger alert pattern: {e}")
            return jsonify({"status": "error", "message": str(e)})
    else:
        return jsonify({"status": "error", "message": "Alert function not available"})

@socketio.on('connect')
def handle_connect():
    """Handle client connection to WebSocket"""
    global client_count, thread
    client_count += 1
    logger.info(f"Client connected. Total clients: {client_count}")
    
    # Start background thread if not already running
    with thread_lock:
        if thread is None:
            thread = Thread(target=background_thread)
            thread.daemon = True
            thread.start()
            logger.info("Background thread started")
    
    # Update client with current status
    emit('status_update', device_status)

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection from WebSocket"""
    global client_count, camera_feed_active
    client_count -= 1
    logger.info(f"Client disconnected. Total clients: {client_count}")
    
    # Stop camera feed if no clients are connected
    if client_count == 0:
        camera_feed_active = False

@socketio.on('start_camera')
def handle_start_camera():
    """Start the camera feed"""
    global camera_feed_active
    camera_feed_active = True
    logger.info("Camera feed started")
    emit('camera_status', {'active': True})

@socketio.on('stop_camera')
def handle_stop_camera():
    """Stop the camera feed"""
    global camera_feed_active
    camera_feed_active = False
    logger.info("Camera feed stopped")
    emit('camera_status', {'active': False})

def generate_camera_frames():
    """Generate camera frames from the walking stick camera"""
    global last_frame, camera_thread
    
    if walking_stick_module and hasattr(walking_stick_module, 'camera') and walking_stick_module.camera and walking_stick_module.camera.isOpened():
        camera = walking_stick_module.camera
        
        while camera_feed_active:
            success, frame = camera.read()
            
            if not success:
                logger.error("Failed to read from camera")
                break
                
            try:
                # Process frame with obstacle detection if model is available
                if walking_stick_module.model:
                    processed_frame, obstacles = walking_stick_module.process_frame(frame)
                    device_status["obstacles"] = obstacles
                    # Use processed frame with detection boxes
                    frame = processed_frame
                
                # Convert frame to JPEG
                _, buffer = cv2.imencode('.jpg', frame)
                frame_bytes = buffer.tobytes()
                
                # Encode as base64 for sending through WebSocket
                frame_base64 = base64.b64encode(frame_bytes).decode('utf-8')
                last_frame = frame_base64
                
                # Send frame to connected clients
                socketio.emit('camera_frame', {'frame': frame_base64})
                
                # Limit frame rate to reduce CPU usage
                time.sleep(0.1)
            except Exception as e:
                logger.error(f"Error in camera frame generation: {e}")
                break
    else:
        # Generate mock camera frames when hardware is not available
        logger.info("Using mock camera frames (no camera hardware detected)")
        width, height = 640, 480
        
        while camera_feed_active:
            try:
                # Create a blank frame
                frame = np.zeros((height, width, 3), dtype=np.uint8)
                
                # Add text and visual elements
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                cv2.putText(frame, "Vision Mate - Mock Camera", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(frame, timestamp, (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                # Add a moving element to simulate motion
                t = time.time()
                x = int(width/2 + width/4 * math.sin(t))
                y = int(height/2 + height/4 * math.cos(t))
                cv2.circle(frame, (x, y), 20, (0, 165, 255), -1)
                
                # Draw mock obstacles
                if device_status["obstacles"]:
                    for obstacle in device_status["obstacles"]:
                        x, y, w, h = obstacle['box']
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        cv2.putText(frame, obstacle['class'], (x, y-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # Add distance reading
                cv2.putText(frame, f"Distance: {device_status['distance']:.1f} cm", 
                            (10, height-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                # Convert frame to JPEG
                _, buffer = cv2.imencode('.jpg', frame)
                frame_bytes = buffer.tobytes()
                
                # Encode as base64 for sending through WebSocket
                frame_base64 = base64.b64encode(frame_bytes).decode('utf-8')
                last_frame = frame_base64
                
                # Send frame to connected clients
                socketio.emit('camera_frame', {'frame': frame_base64})
                
                # Limit frame rate
                time.sleep(0.1)
            except Exception as e:
                logger.error(f"Error generating mock camera frames: {e}")
                time.sleep(1)

# Mock data for testing when hardware is not available
def generate_mock_data():
    """Generate mock data for testing when hardware is not available"""
    # Mock distance (50-300 cm with some variation)
    distance = 150 + 50 * math.sin(datetime.now().timestamp() / 5)
    
    # Mock accelerometer data
    accel_x = random.uniform(-0.2, 0.2)
    accel_y = random.uniform(-0.2, 0.2)
    accel_z = 1.0 + random.uniform(-0.1, 0.1)  # Mostly 1g downward
    
    # Occasional mock fall detection (1% chance)
    fall_detected = random.random() < 0.01
    
    # Mock GPS location (small random walk from fixed point)
    lat = 37.7749 + random.uniform(-0.001, 0.001)
    lon = -122.4194 + random.uniform(-0.001, 0.001)
    
    # Mock obstacles (10% chance of having 1-3 obstacles)
    obstacles = []
    if random.random() < 0.1:
        num_obstacles = random.randint(1, 3)
        for _ in range(num_obstacles):
            class_names = ["person", "car", "chair", "dog", "bicycle", "fire hydrant"]
            obstacles.append({
                'class': random.choice(class_names),
                'position': random.choice(["left", "right", "center"]),
                'estimated_distance': random.uniform(100, 400),
                'box': [random.randint(10, 100), random.randint(10, 100), 
                        random.randint(50, 200), random.randint(50, 200)]
            })
    
    # Mock GPIO states (randomly change states occasionally)
    gpio_status = device_status["gpio_status"].copy()
    if random.random() < 0.02:  # 2% chance of LED change
        # Randomly select one LED to toggle
        led_choice = random.choice(["led_red", "led_green", "led_blue"])
        gpio_status[led_choice] = not gpio_status[led_choice]
    
    return {
        "distance": distance,
        "accelerometer": {"x": accel_x, "y": accel_y, "z": accel_z},
        "fall_detected": fall_detected,
        "gps_location": {"lat": lat, "lon": lon},
        "obstacles": obstacles,
        "gpio_status": gpio_status
    }

def background_thread():
    """Background thread to update status continuously"""
    while True:
        if client_count <= 0:
            time.sleep(1)
            continue
            
        try:
            if walking_stick_module:
                # Update distance
                if hasattr(walking_stick_module, 'get_distance'):
                    device_status["distance"] = walking_stick_module.get_distance()
                
                # Update GPS location
                if hasattr(walking_stick_module, 'get_gps_location'):
                    lat, lon = walking_stick_module.get_gps_location()
                    device_status["gps_location"] = {"lat": lat, "lon": lon}
                
                # Update accelerometer data and fall detection
                if hasattr(walking_stick_module, 'mpu') and walking_stick_module.mpu:
                    accel_data = walking_stick_module.mpu.get_accel_data()
                    device_status["accelerometer"] = accel_data
                    
                    # Check for falls
                    if hasattr(walking_stick_module, 'detect_fall'):
                        device_status["fall_detected"] = walking_stick_module.detect_fall()
                
                # Update component status
                if hasattr(walking_stick_module, 'camera'):
                    device_status["components_status"]["camera"] = (
                        walking_stick_module.camera is not None and 
                        walking_stick_module.camera.isOpened()
                    )
                
                if hasattr(walking_stick_module, 'mpu'):
                    device_status["components_status"]["accelerometer"] = walking_stick_module.mpu is not None
                
                if hasattr(walking_stick_module, 'bluetooth_serial'):
                    device_status["components_status"]["bluetooth"] = walking_stick_module.bluetooth_serial is not None
                
                if hasattr(walking_stick_module, 'model'):
                    device_status["components_status"]["model"] = walking_stick_module.model is not None
                
                # Update GPIO pin status
                if hasattr(walking_stick_module, 'GPIO'):
                    try:
                        device_status["gpio_status"]["led_red"] = (
                            walking_stick_module.GPIO.input(walking_stick_module.LED_RED_PIN) == walking_stick_module.GPIO.HIGH
                        )
                        device_status["gpio_status"]["led_green"] = (
                            walking_stick_module.GPIO.input(walking_stick_module.LED_GREEN_PIN) == walking_stick_module.GPIO.HIGH
                        )
                        device_status["gpio_status"]["led_blue"] = (
                            walking_stick_module.GPIO.input(walking_stick_module.LED_BLUE_PIN) == walking_stick_module.GPIO.HIGH
                        )
                        device_status["gpio_status"]["buzzer"] = (
                            walking_stick_module.GPIO.input(walking_stick_module.BUZZER_PIN) == walking_stick_module.GPIO.HIGH
                        )
                        # Vibration motor status is harder to check with PWM, so we'll skip it for now
                    except Exception as e:
                        logger.error(f"Error reading GPIO status: {e}")
                
                device_status["connected"] = True
            else:
                # Generate mock data when hardware is not available
                mock_data = generate_mock_data()
                device_status.update(mock_data)
                device_status["components_status"]["camera"] = True
                device_status["components_status"]["accelerometer"] = True
                device_status["components_status"]["gps"] = True
                device_status["components_status"]["bluetooth"] = True
                device_status["components_status"]["model"] = True
                device_status["connected"] = True
                logger.info("Using mock data (no hardware detected)")
            
            # Send status update to connected clients
            socketio.emit('status_update', device_status)
            
            # Handle camera feed if requested
            if camera_feed_active and device_status["components_status"]["camera"]:
                with thread_lock:
                    if camera_thread is None or not camera_thread.is_alive():
                        camera_thread = Thread(target=generate_camera_frames)
                        camera_thread.daemon = True
                        camera_thread.start()
            
            # Sleep to reduce update frequency
            time.sleep(0.5)
        except Exception as e:
            logger.error(f"Error in background thread: {e}")
            time.sleep(1)  # Wait a bit before retrying

if __name__ == '__main__':
    # Create templates and static directories if they don't exist
    os.makedirs('templates', exist_ok=True)
    os.makedirs('static', exist_ok=True)
    os.makedirs('static/css', exist_ok=True)
    os.makedirs('static/js', exist_ok=True)
    
    # Start the Flask app with SocketIO
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, allow_unsafe_werkzeug=True) 