# Vision Mate - Smart Walking Stick Web Interface

This project provides a web interface for connecting to and controlling a smart walking stick device. The web interface displays real-time data from the walking stick's sensors and allows for remote control of various features.

## Features

- Real-time camera feed with object detection visualization
- GPS location tracking on an interactive map
- Ultrasonic distance sensor readings
- Accelerometer data with fall detection
- Emergency alert triggering
- Voice message sending through the walking stick speaker
- Component status monitoring

## Requirements

### For Raspberry Pi (Full Hardware Support)
- Python 3.6 or higher
- Raspberry Pi running the walking stick software
- Web browser with JavaScript enabled
- Internet connection for map functionality

### For Windows Development (Simulation Mode)
- Python 3.6 or higher
- Web browser with JavaScript enabled
- Internet connection for map functionality

## Hardware Requirements (Raspberry Pi 4 B)
- Raspberry Pi 4 Model B
- MPU6050 3-Axis Gyroscope/Accelerometer
- HC-SR04 Ultrasonic Distance Sensor
- RGB LED
- Buzzer
- Vibration Motor
- Emergency Button
- Raspberry Pi Camera Module V2 or better
- GPS Module (Compatible with GPSD)
- Power supply (5V/3A recommended)
- microSD card (16GB or larger)

## Pin Configuration
- MPU6050: 
  - SDA → GPIO2 (Pin 3)
  - SCL → GPIO3 (Pin 5)
  - VCC → 3.3V
  - GND → Ground
- Ultrasonic Sensor:
  - TRIG → GPIO23
  - ECHO → GPIO24
  - VCC → 5V
  - GND → Ground
- RGB LED:
  - Red → GPIO5
  - Green → GPIO6
  - Blue → GPIO13
  - GND → Ground
- Other Components:
  - Button → GPIO27
  - Buzzer → GPIO22
  - Vibration Motor → GPIO18

## Installation

### Prerequisites
- Python 3.6 or higher
- pip package manager
- Git (optional)

### Windows Development Setup
1. Create and activate virtual environment:
```bash
python -m venv venv
venv\Scripts\activate
```

2. Install Windows-specific requirements:
```bash
pip install -r requirements-windows.txt
```

3. If you encounter module issues, try installing core dependencies individually:
```bash
pip install flask flask-socketio opencv-python numpy pillow python-engineio python-socketio pyttsx3
```

4. Verify installations:
```bash
python -c "import flask; import flask_socketio; import cv2; import numpy; import pyttsx3"
```

### Raspberry Pi Setup
1. Run the setup script:
```bash
chmod +x setup_pi.sh
./setup_pi.sh
```

2. If modules are still missing after setup:
```bash
source venv/bin/activate
pip install --no-cache-dir -r requirements.txt
```

### On Raspberry Pi 4 B

1. Install Raspberry Pi OS (64-bit recommended):
   ```bash
   # Download and flash Raspberry Pi OS to your SD card using Raspberry Pi Imager
   # Enable SSH, I2C, and Camera interfaces during installation
   ```

2. Initial Setup:
   ```bash
   # Update system packages
   sudo apt-get update
   sudo apt-get upgrade -y
   
   # Install git
   sudo apt-get install git -y
   ```

3. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/vision-mate.git
   cd vision-mate
   ```

4. Run the setup script:
   ```bash
   chmod +x setup_pi.sh
   ./setup_pi.sh
   ```

5. Reboot the Raspberry Pi:
   ```bash
   sudo reboot
   ```

6. After reboot, activate the virtual environment:
   ```bash
   cd vision-mate
   source venv/bin/activate
   ```

7. Configure YOLO:
   ```bash
   # Download YOLO tiny weights and config
   wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights
   wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg
   wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/data/coco.names
   ```

8. Test the hardware:
   ```bash
   # Check I2C devices (should see address 0x68 for MPU6050)
   sudo i2cdetect -y 1
   
   # Test camera
   raspistill -o test.jpg
   ```

9. Start the application:
   ```bash
   python app.py
   ```

### On Windows (development/simulation)

1. Clone this repository to your Windows machine:

```bash
git clone https://github.com/yourusername/vision-mate.git
cd vision-mate
```

2. Install the Windows-specific requirements:

```bash
pip install -r requirements-windows.txt
```

3. (Optional) If you want to use Google Maps for location visualization, edit the API key in `templates/index.html`:

```html
<script src="https://maps.googleapis.com/maps/api/js?key=YOUR_API_KEY&callback=initMap" async defer></script>
```

## Usage

1. Start the web server:

```bash
python app.py
```

2. Open a web browser and navigate to:

```
http://localhost:5000
```

3. Use the web interface to:
   - View the camera feed by toggling the "Enable Camera" switch
   - Monitor sensor data in real-time
   - Send voice messages through the walking stick
   - Trigger emergency alerts
   - Track the walking stick's location

## Development Mode

When running on a system without the actual hardware (like a Windows development machine), the application will automatically use simulation mode:

- Mock sensor data will be generated
- A simulated camera feed will be displayed
- All functionality can be tested without actual hardware

## Integrating with the Walking Stick

The web interface connects directly to the Walking Stick Python script. The `app.py` file imports the walking stick module and accesses its functions and data. The walking stick script should be in the same directory as the web application files.

## Troubleshooting

### Common Issues

1. Module Not Found Errors
   - Ensure virtual environment is activated
   - Try reinstalling the specific module
   - Check Python path with: `python -c "import sys; print(sys.path)"`

2. OpenCV Issues
   - Windows: `pip install opencv-python`
   - Raspberry Pi: `pip install opencv-python-headless`

3. GPIO Access (Raspberry Pi)
   - Run with sudo if needed
   - Verify user is in gpio group: `sudo usermod -a -G gpio $USER`

4. Camera Access
   - Check camera connection
   - Enable camera interface (Raspberry Pi)
   - Test camera: `python -c "import cv2; cap=cv2.VideoCapture(0)"`

5. Audio Issues
   - Install portaudio: `sudo apt-get install portaudio19-dev`
   - Install pyttsx3: `pip install pyttsx3`

### Environment Setup Tips

1. Virtual Environment
```bash
# If venv creation fails:
python -m pip install --upgrade virtualenv
python -m virtualenv venv

# If activation fails on Windows:
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

2. Package Management
```bash
# Clear pip cache
pip cache purge

# Force reinstall packages
pip install --force-reinstall -r requirements.txt
```

3. System Checks
```bash
# Check Python version
python --version

# Check pip version
pip --version

# List installed packages
pip list
```

## Security Considerations

- This application is designed for use on a local network. Exposing it to the internet would require additional security measures like HTTPS, authentication, etc.
- The emergency contact feature requires proper configuration in the walking stick software.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- This project uses Flask, a lightweight WSGI web application framework for Python
- Socket.IO is used for real-time communication
- OpenCV is used for computer vision features
- Bootstrap is used for the UI components

## Hardware Verification

Before running the application, verify:
1. MPU6050 is detected on I2C bus
2. Camera module is properly connected
3. All GPIO pins are correctly wired
4. GPS module is recognized by the system
5. Bluetooth audio device is paired (if using)

## Bluetooth Audio Setup

1. Pair your Bluetooth headset:
   ```bash
   sudo bluetoothctl
   # In bluetoothctl:
   scan on
   pair [MAC_ADDRESS]
   trust [MAC_ADDRESS]
   connect [MAC_ADDRESS]
   quit
   ```

2. Test audio output:
   ```bash
   # Play test sound through Bluetooth
   aplay -D bluealsa:HCI=hci0,DEV=[MAC_ADDRESS],PROFILE=a2dp test.wav
   ```