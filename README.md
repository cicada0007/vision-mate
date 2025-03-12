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

## Installation

### On Raspberry Pi (with hardware)

1. Clone this repository to your Raspberry Pi:

```bash
git clone https://github.com/yourusername/vision-mate.git
cd vision-mate
```

2. Install the required Python packages:

```bash
pip install -r requirements.txt
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

- If the camera feed doesn't appear, check that the camera is properly connected to the Raspberry Pi and that the walking stick software can access it.
- If sensor data is not updating, verify that the respective sensors are properly connected and initialized.
- If the web interface shows "Disconnected", ensure that the application is running and that there are no network issues between your browser and the server.
- On Windows, if you encounter issues with specific packages, try installing them individually with pip.

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