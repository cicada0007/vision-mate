// Main JavaScript for Walking Stick Web Interface

// Global variables
let socket;
let map;
let marker;
let connected = false;
let cameraActive = false;

// Initialize once the document is fully loaded
document.addEventListener('DOMContentLoaded', function() {
    initSocket();
    setupEventListeners();
});

// Initialize WebSocket connection
function initSocket() {
    // Connection options for threading mode
    const socketOptions = {
        transports: ['websocket', 'polling'],
        forceNew: true,
        reconnection: true,
        reconnectionAttempts: 5,
        reconnectionDelay: 1000
    };
    
    // Connect to the server
    socket = io.connect(location.protocol + '//' + document.domain + ':' + location.port, socketOptions);
    
    // Socket connection established
    socket.on('connect', function() {
        console.log('Connected to server');
        updateConnectionStatus(true);
    });
    
    // Socket disconnected
    socket.on('disconnect', function() {
        console.log('Disconnected from server');
        updateConnectionStatus(false);
    });
    
    // Handle status updates from the walking stick
    socket.on('status_update', function(data) {
        updateDeviceStatus(data);
    });
    
    // Handle camera frames
    socket.on('camera_frame', function(data) {
        updateCameraFeed(data.frame);
    });
    
    // Handle camera status updates
    socket.on('camera_status', function(data) {
        updateCameraStatus(data.active);
    });
}

// Set up UI event listeners
function setupEventListeners() {
    // Camera toggle
    document.getElementById('camera-toggle').addEventListener('change', function(e) {
        if (e.target.checked) {
            startCamera();
        } else {
            stopCamera();
        }
    });
    
    // Emergency button
    document.getElementById('emergency-button').addEventListener('click', function() {
        triggerEmergency();
    });
    
    // Speak button
    document.getElementById('speak-button').addEventListener('click', function() {
        sendSpeakMessage();
    });
    
    // Enter key in speak message field
    document.getElementById('speak-message').addEventListener('keypress', function(e) {
        if (e.key === 'Enter') {
            sendSpeakMessage();
        }
    });
}

// Update connection status in the UI
function updateConnectionStatus(isConnected) {
    connected = isConnected;
    const statusElement = document.getElementById('connection-status');
    
    if (isConnected) {
        statusElement.textContent = 'Connected';
        statusElement.classList.remove('bg-danger');
        statusElement.classList.add('bg-success', 'connected');
    } else {
        statusElement.textContent = 'Disconnected';
        statusElement.classList.remove('bg-success', 'connected');
        statusElement.classList.add('bg-danger');
        
        // Reset all component statuses
        updateComponentStatus('camera', false);
        updateComponentStatus('accelerometer', false);
        updateComponentStatus('gps', false);
        updateComponentStatus('bluetooth', false);
        updateComponentStatus('model', false);
        
        // Hide fall alert if showing
        document.getElementById('fall-alert').classList.add('d-none');
    }
}

// Update device status in the UI
function updateDeviceStatus(data) {
    // Update distance indicator
    const distance = data.distance;
    const distanceBar = document.getElementById('distance-bar');
    const distanceValue = document.getElementById('distance-value');
    
    // Calculate percentage (max 500cm)
    const percentage = Math.min(100, (distance / 500) * 100);
    distanceBar.style.width = percentage + '%';
    distanceValue.textContent = distance.toFixed(1) + ' cm';
    
    // Update accelerometer values
    if (data.accelerometer) {
        updateAccelerometerUI(data.accelerometer);
    }
    
    // Update GPS location
    if (data.gps_location && data.gps_location.lat && data.gps_location.lon) {
        updateGPSLocation(data.gps_location.lat, data.gps_location.lon);
    }
    
    // Update fall detection status
    if (data.fall_detected) {
        document.getElementById('fall-alert').classList.remove('d-none');
    } else {
        document.getElementById('fall-alert').classList.add('d-none');
    }
    
    // Update obstacles list
    if (data.obstacles && data.obstacles.length > 0) {
        updateObstaclesList(data.obstacles);
    } else {
        document.getElementById('no-obstacles').classList.remove('d-none');
        document.getElementById('obstacles-list').innerHTML = '';
    }
    
    // Update component statuses
    if (data.components_status) {
        updateComponentStatus('camera', data.components_status.camera);
        updateComponentStatus('accelerometer', data.components_status.accelerometer);
        updateComponentStatus('gps', data.components_status.gps);
        updateComponentStatus('bluetooth', data.components_status.bluetooth);
        updateComponentStatus('model', data.components_status.model);
    }
}

// Update accelerometer data in UI
function updateAccelerometerUI(accelData) {
    // X-axis
    const xBar = document.getElementById('accel-x-bar');
    const xValue = document.getElementById('accel-x-value');
    const xPercentage = ((accelData.x + 2) / 4) * 100; // Range from -2g to +2g
    xBar.style.width = xPercentage + '%';
    xValue.textContent = accelData.x.toFixed(2) + ' g';
    
    // Y-axis
    const yBar = document.getElementById('accel-y-bar');
    const yValue = document.getElementById('accel-y-value');
    const yPercentage = ((accelData.y + 2) / 4) * 100;
    yBar.style.width = yPercentage + '%';
    yValue.textContent = accelData.y.toFixed(2) + ' g';
    
    // Z-axis
    const zBar = document.getElementById('accel-z-bar');
    const zValue = document.getElementById('accel-z-value');
    const zPercentage = ((accelData.z + 2) / 4) * 100;
    zBar.style.width = zPercentage + '%';
    zValue.textContent = accelData.z.toFixed(2) + ' g';
}

// Update component status in UI
function updateComponentStatus(component, isOnline) {
    const statusElement = document.getElementById(`${component}-status`);
    
    if (isOnline) {
        statusElement.textContent = 'Online';
        statusElement.classList.remove('bg-danger');
        statusElement.classList.add('bg-success');
    } else {
        statusElement.textContent = 'Offline';
        statusElement.classList.remove('bg-success');
        statusElement.classList.add('bg-danger');
    }
}

// Update camera feed with received frame
function updateCameraFeed(frameBase64) {
    const cameraFeed = document.getElementById('camera-feed');
    const cameraPlaceholder = document.getElementById('camera-placeholder');
    
    cameraFeed.src = 'data:image/jpeg;base64,' + frameBase64;
    cameraFeed.classList.remove('d-none');
    cameraPlaceholder.classList.add('d-none');
}

// Update camera status
function updateCameraStatus(isActive) {
    cameraActive = isActive;
    document.getElementById('camera-toggle').checked = isActive;
    
    if (!isActive) {
        const cameraFeed = document.getElementById('camera-feed');
        const cameraPlaceholder = document.getElementById('camera-placeholder');
        
        cameraFeed.classList.add('d-none');
        cameraPlaceholder.classList.remove('d-none');
    }
}

// Start camera feed
function startCamera() {
    if (connected) {
        socket.emit('start_camera');
    }
}

// Stop camera feed
function stopCamera() {
    if (connected) {
        socket.emit('stop_camera');
    }
}

// Trigger emergency alert
function triggerEmergency() {
    if (!connected) {
        alert('Cannot trigger emergency - device not connected');
        return;
    }
    
    // Confirm before sending emergency alert
    if (confirm('Are you sure you want to trigger an emergency alert?')) {
        fetch('/api/emergency', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        })
        .then(response => response.json())
        .then(data => {
            if (data.status === 'success') {
                alert('Emergency alert triggered successfully!');
            } else {
                alert('Failed to trigger emergency: ' + data.message);
            }
        })
        .catch(error => {
            console.error('Error:', error);
            alert('Failed to trigger emergency: Network error');
        });
    }
}

// Send speak message
function sendSpeakMessage() {
    const messageInput = document.getElementById('speak-message');
    const message = messageInput.value.trim();
    
    if (!connected) {
        alert('Cannot send message - device not connected');
        return;
    }
    
    if (message) {
        fetch('/api/speak', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ message: message })
        })
        .then(response => response.json())
        .then(data => {
            if (data.status === 'success') {
                messageInput.value = '';
            } else {
                alert('Failed to send message: ' + data.message);
            }
        })
        .catch(error => {
            console.error('Error:', error);
            alert('Failed to send message: Network error');
        });
    }
}

// Update obstacles list
function updateObstaclesList(obstacles) {
    const obstaclesList = document.getElementById('obstacles-list');
    const noObstacles = document.getElementById('no-obstacles');
    
    obstaclesList.innerHTML = '';
    
    if (obstacles.length > 0) {
        noObstacles.classList.add('d-none');
        
        obstacles.forEach(obstacle => {
            const obstacleItem = document.createElement('div');
            obstacleItem.className = 'col-md-4';
            
            obstacleItem.innerHTML = `
                <div class="obstacle-item">
                    <div class="d-flex align-items-center">
                        <i class="fas fa-exclamation-triangle obstacle-icon"></i>
                        <div>
                            <h5 class="mb-1">${obstacle.class}</h5>
                            <p class="mb-0">Position: ${obstacle.position}</p>
                            <p class="mb-0">Est. Distance: ${obstacle.estimated_distance.toFixed(1)} cm</p>
                        </div>
                    </div>
                </div>
            `;
            
            obstaclesList.appendChild(obstacleItem);
        });
    } else {
        noObstacles.classList.remove('d-none');
    }
}

// Update GPS location on map
function updateGPSLocation(lat, lon) {
    document.getElementById('latitude').textContent = lat.toFixed(6);
    document.getElementById('longitude').textContent = lon.toFixed(6);
    
    if (map && marker) {
        const position = new google.maps.LatLng(lat, lon);
        marker.setPosition(position);
        map.setCenter(position);
    }
}

// Initialize Google Maps
function initMap() {
    // Default location (will be updated with actual GPS coordinates)
    const defaultLocation = { lat: 0, lng: 0 };
    
    map = new google.maps.Map(document.getElementById('map'), {
        zoom: 17,
        center: defaultLocation,
        mapTypeId: 'roadmap',
        styles: [
            {
                featureType: 'poi',
                elementType: 'labels',
                stylers: [{ visibility: 'off' }]
            }
        ]
    });
    
    marker = new google.maps.Marker({
        position: defaultLocation,
        map: map,
        title: 'Walking Stick Location',
        icon: {
            url: 'https://maps.google.com/mapfiles/ms/icons/blue-dot.png'
        }
    });
} 