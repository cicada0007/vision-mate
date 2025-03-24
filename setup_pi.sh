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

# Create Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install Python dependencies with error handling
echo "Installing Python dependencies..."
pip install --upgrade pip

# Install packages one by one to better handle errors
PACKAGES=(
    "flask>=2.0.1"
    "flask-socketio>=5.1.1"
    "opencv-python-headless>=4.5.3"
    "numpy>=1.21.2"
    "RPi.GPIO==0.7.0"
    "gpsd-py3==0.3.0"
    "smbus2==0.4.1"
    "pyttsx3==2.90"
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
    pip install $package || echo "Warning: Failed to install $package"
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
