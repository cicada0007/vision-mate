#!/bin/bash

# Update system
sudo apt-get update
sudo apt-get upgrade -y

# Install system dependencies
sudo apt-get install -y \
    python3-pip \
    python3-venv \
    i2c-tools \
    libi2c-dev \
    python3-smbus \
    libgpiod2 \
    libatlas-base-dev \
    libhdf5-dev \
    libhdf5-serial-dev \
    libjasper-dev \
    libqtgui4 \
    libqt4-test \
    libportaudio2 \
    portaudio19-dev \
    git

# Enable I2C interface
if ! grep -q "^dtparam=i2c_arm=on" /boot/config.txt; then
    echo "dtparam=i2c_arm=on" | sudo tee -a /boot/config.txt
fi

# Enable I2C in modules
if ! grep -q "^i2c-dev" /etc/modules; then
    echo "i2c-dev" | sudo tee -a /etc/modules
fi

# Enable camera interface
if ! grep -q "^start_x=1" /boot/config.txt; then
    echo "start_x=1" | sudo tee -a /boot/config.txt
fi

# Add hardware verification steps before package installation
echo "Verifying hardware connections..."

# Check I2C connection
echo "Checking I2C connection..."
if ! i2cdetect -y 1 | grep -q "68"; then
    echo "Warning: MPU6050 not detected on I2C bus!"
    echo "Please check:"
    echo "1. I2C is enabled (already done in setup)"
    echo "2. MPU6050 connections:"
    echo "   - SDA → GPIO2 (Pin 3)"
    echo "   - SCL → GPIO3 (Pin 5)"
    echo "   - VCC → 3.3V"
    echo "   - GND → Ground"
fi

# Check GPIO access
echo "Checking GPIO access..."
if ! groups | grep -q "gpio"; then
    echo "Adding user to gpio group..."
    sudo usermod -a -G gpio $USER
    echo "Please log out and log back in for GPIO access"
fi

# Check camera module
echo "Checking camera module..."
if ! vcgencmd get_camera | grep -q "supported=1 detected=1"; then
    echo "Warning: Camera module not detected!"
    echo "Please check:"
    echo "1. Camera interface is enabled (already done in setup)"
    echo "2. Camera ribbon cable is properly connected"
    echo "3. Camera module is compatible with Raspberry Pi"
fi

# Hardware verification report
echo "Hardware Connection Guide:"
echo "1. MPU6050 Accelerometer:"
echo "   - SDA → GPIO2 (Pin 3)"
echo "   - SCL → GPIO3 (Pin 5)"
echo "   - VCC → 3.3V"
echo "   - GND → Ground"
echo "2. Ultrasonic Sensor HC-SR04:"
echo "   - TRIG → GPIO23"
echo "   - ECHO → GPIO24"
echo "   - VCC → 5V"
echo "   - GND → Ground"
echo "3. RGB LED:"
echo "   - Red → GPIO5"
echo "   - Green → GPIO6"
echo "   - Blue → GPIO13"
echo "   - GND → Ground (via resistor)"
echo "4. Other Components:"
echo "   - Button → GPIO27 (with pull-up)"
echo "   - Buzzer → GPIO22"
echo "   - Vibration Motor → GPIO18"
echo "   - All GND pins must connect to ground"

# Create hardware test script
cat > test_hardware.py << 'EOF'
import RPi.GPIO as GPIO
import time
import smbus2
import cv2

def test_mpu6050():
    try:
        bus = smbus2.SMBus(1)
        bus.write_byte_data(0x68, 0x6B, 0)
        print("MPU6050: Connected")
        return True
    except Exception as e:
        print(f"MPU6050 Error: {e}")
        return False

def test_gpio():
    try:
        GPIO.setmode(GPIO.BCM)
        test_pins = [5, 6, 13, 22, 23, 24, 27, 18]
        for pin in test_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(0.2)
            GPIO.output(pin, GPIO.LOW)
        print("GPIO: Working")
        return True
    except Exception as e:
        print(f"GPIO Error: {e}")
        return False

def test_camera():
    try:
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret:
                print("Camera: Working")
                return True
        print("Camera: Not detected")
        return False
    except Exception as e:
        print(f"Camera Error: {e}")
        return False

if __name__ == "__main__":
    print("Testing hardware connections...")
    mpu = test_mpu6050()
    gpio = test_gpio()
    camera = test_camera()
    
    print("\nHardware Test Results:")
    print(f"MPU6050: {'✓' if mpu else '✗'}")
    print(f"GPIO: {'✓' if gpio else '✗'}")
    print(f"Camera: {'✓' if camera else '✗'}")
    
    if not all([mpu, gpio, camera]):
        print("\nSome hardware tests failed. Please check connections.")
EOF

chmod +x test_hardware.py
echo "Hardware test script created. Run 'python test_hardware.py' after setup to verify connections."

# Create Python virtual environment with better error handling
echo "Creating Python virtual environment..."
python3 -m pip install --upgrade virtualenv || {
    echo "Failed to install virtualenv. Trying with pip3..."
    pip3 install virtualenv
}

# Remove existing venv if it exists
rm -rf venv

# Create new virtual environment with error handling
python3 -m virtualenv venv || {
    echo "Failed to create venv with python3. Trying alternative method..."
    virtualenv venv
}

# Activate virtual environment with error checking
if [ -f "venv/bin/activate" ]; then
    source venv/bin/activate
else
    echo "Error: Virtual environment activation script not found!"
    exit 1
fi

# Verify activation
if [ -z "$VIRTUAL_ENV" ]; then
    echo "Error: Virtual environment not activated properly!"
    exit 1
fi

# Update pip with error handling
echo "Updating pip..."
python -m pip install --upgrade pip || {
    echo "Failed to upgrade pip. Trying alternative method..."
    curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
    python get-pip.py
    rm get-pip.py
}

# Install packages with better error handling and updated versions
echo "Installing Python dependencies..."
PACKAGES=(
    "flask>=2.3.0"
    "flask-socketio>=5.3.0"
    "opencv-python-headless>=4.7.0"
    "numpy>=1.24.0"
    "RPi.GPIO>=0.7.1"
    "gpsd-py3>=0.3.0"
    "smbus2>=0.4.2"
    "pyttsx3>=2.90"
    "pillow>=8.3.1"
    "gunicorn==20.1.0"
    "python-engineio>=4.3.1"
    "python-socketio>=5.5.2"
    "sounddevice>=0.4.4"
    "picamera2>=0.3.12"
    "i2c-tools>=4.3"
    "gpiozero>=1.6.2"
    "adafruit-blinka>=6.0.0"
)

for package in "${PACKAGES[@]}"; do
    echo "Installing $package..."
    python -m pip install $package || {
        echo "Failed to install $package. Trying alternative method..."
        python -m pip install --no-cache-dir $package || {
            echo "Warning: Failed to install $package after retry"
            echo $package >> failed_packages.txt
        }
    }
done

# Verify Flask installation
python -c "import flask" || {
    echo "Flask installation failed. Trying alternative method..."
    pip install flask --no-cache-dir
}

# Install OpenCV dependencies
sudo apt-get install -y \
    python3-opencv \
    libopencv-dev

# Verify critical packages
python - <<EOF
import sys
packages = ['flask', 'flask_socketio', 'cv2', 'numpy', 'pyttsx3']
missing = []
for package in packages:
    try:
        __import__(package)
    except ImportError:
        missing.append(package)
if missing:
    print(f"Warning: The following packages are still missing: {', '.join(missing)}")
    sys.exit(1)
print("All critical packages verified!")
EOF

echo "Setup complete! Please reboot your Raspberry Pi"
echo "After reboot, activate the virtual environment with: source venv/bin/activate"
