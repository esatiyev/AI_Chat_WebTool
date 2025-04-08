// Initialize Socket.IO connection
const socket = io('http://localhost:5000');

// State management
const state = {
    connected: false,
    vehicleArmed: false,
    position: { lat: 0, lon: 0, alt: 0, heading: 0 },
    attitude: { roll: 0, pitch: 0, yaw: 0 },
    battery: { voltage: 0, current: 0, remaining: 0 },
    gps: { status: 0, satellites: 0, hdop: 0 },
    commandHistory: [],
    historyIndex: -1
};

// Initialize Map
const map = L.map('map').setView([0, 0], 2);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);

// Create SVG icon for drone marker
const droneIcon = L.icon({
    iconUrl: 'data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="%23e74c3c"><circle cx="12" cy="12" r="10"/><path d="M12 2v20" stroke="white" stroke-width="2"/><path d="M12 2l6 6" stroke="white" stroke-width="2"/><path d="M12 2l-6 6" stroke="white" stroke-width="2"/></svg>',
    iconSize: [24, 24],
    iconAnchor: [12, 12]
});

// Create rotated marker
const droneMarker = L.marker([0, 0], {
    icon: droneIcon,
    rotationAngle: 0,
    rotationOrigin: 'center'
}).addTo(map);

// Helper functions
function showToast(message, type = 'info') {
    const toast = document.createElement('div');
    toast.className = `toast toast-${type}`;
    toast.textContent = message;
    document.body.appendChild(toast);

    setTimeout(() => {
        toast.classList.add('fade-out');
        setTimeout(() => toast.remove(), 500);
    }, 3000);
}

function addChatMessage(message, sender) {
    const chat = document.getElementById('chat-messages');
    const msgDiv = document.createElement('div');
    msgDiv.className = sender === 'user' ? 'chat-message-user' : 'chat-message-bot';

    // Preserve line breaks in messages
    const formattedMessage = message.replace(/\n/g, '<br>');
    msgDiv.innerHTML = formattedMessage;

    chat.appendChild(msgDiv);
    chat.scrollTop = chat.scrollHeight;
}

function updateAttitudeIndicator(roll, pitch) {
    const ball = document.getElementById('attitude-ball');
    const maxTilt = 30; // degrees

    // Normalize values
    const normRoll = Math.min(Math.max(roll / maxTilt, -1), 1);
    const normPitch = Math.min(Math.max(pitch / maxTilt, -1), 1);

    ball.style.transform = `translate(-50%, -50%) 
translateX(${normRoll * 30}px) 
translateY(${normPitch * 30}px) 
rotate(${roll}deg)`;
}

function updateCommandHistory(command) {
    if (!command) return;

    // Add to history if not the same as last command
    if (state.commandHistory.length === 0 ||
        state.commandHistory[state.commandHistory.length - 1] !== command) {
        state.commandHistory.push(command);

        // Keep only last 10 commands
        if (state.commandHistory.length > 10) {
            state.commandHistory.shift();
        }
    }

    state.historyIndex = -1;
    renderCommandHistory();
}

function renderCommandHistory() {
    const historyContainer = document.getElementById('command-history');
    historyContainer.innerHTML = '';

    if (state.commandHistory.length === 0) {
        historyContainer.classList.add('hidden');
        return;
    }

    historyContainer.classList.remove('hidden');

    state.commandHistory.slice().reverse().forEach((cmd, idx) => {
        const item = document.createElement('div');
        item.className = 'command-history-item text-sm text-gray-600';
        item.textContent = cmd;
        item.addEventListener('click', () => {
            document.getElementById('message-input').value = cmd;
            document.getElementById('message-input').focus();
        });
        historyContainer.appendChild(item);
    });
}

// Socket.IO event handlers
socket.on('connect', () => {
    showToast('Connected to control server', 'success');
    addChatMessage('Connected to control server', 'bot');
});

socket.on('disconnect', () => {
    showToast('Disconnected from server', 'warning');
    addChatMessage('Disconnected from server', 'bot');
    updateConnectionStatus(false);
});

socket.on('connection_status', (data) => {
    updateConnectionStatus(data.connected, data);
    if (data.connected) {
        showToast(`Vehicle connected (${data.vehicle_type})`, 'success');
        addChatMessage(`Vehicle connection established (${data.vehicle_type})`, 'bot');
    }
});

socket.on('connection_error', (data) => {
    showToast(data.message, 'error');
    addChatMessage(`Connection error: ${data.message}`, 'bot');
    updateConnectionStatus(false);
});

socket.on('flight_mode_update', (data) => {
    document.getElementById('flight-mode').textContent = data.mode;
    state.flightMode = data.mode;
});

socket.on('armed_state', (data) => {
    state.vehicleArmed = data.armed;
    const armedState = document.getElementById('armed-state');
    armedState.textContent = data.status;
    armedState.className = data.armed ?
        'text-xl font-bold text-green-600' : 'text-xl font-bold text-red-600';
});

socket.on('position_update', (data) => {
    if (data.lat === undefined || data.lon === undefined ||
        isNaN(data.lat) || isNaN(data.lon)) {
        console.error("Invalid position data:", data);
        return;
    }

    const newPos = [data.lat, data.lon];
    droneMarker.setLatLng(newPos);

    // Only update rotation if heading is provided
    if (typeof data.heading === 'number') {
        if (droneMarker.setRotationAngle) {
            droneMarker.setRotationAngle(data.heading);
        } else {
            // Fallback rotation if plugin not available
            const el = droneMarker.getElement();
            if (el) el.style.transform = `rotate(${data.heading}deg)`;
        }
    }

    // Update position displays
    document.getElementById('lat-value').textContent = data.lat.toFixed(6);
    document.getElementById('lon-value').textContent = data.lon.toFixed(6);
    document.getElementById('alt-value').textContent = `${(data.alt || 0).toFixed(1)} m`;

    // Center map if we have valid data
    if (data.lat !== 0 && data.lon !== 0) {
        if (!map.getBounds().contains(newPos)) {
            map.flyTo(newPos, map.getZoom());
        }
    }
});

socket.on('attitude_update', (data) => {
    state.attitude = {
        roll: data.roll,
        pitch: data.pitch,
        yaw: data.yaw
    };

    updateAttitudeIndicator(data.roll, data.pitch);
    document.getElementById('roll-value').textContent = `${data.roll.toFixed(1)}°`;
    document.getElementById('pitch-value').textContent = `${data.pitch.toFixed(1)}°`;
    document.getElementById('yaw-value').textContent = `${data.yaw.toFixed(1)}°`;
});

socket.on('battery_update', (data) => {
    state.battery = {
        voltage: data.voltage,
        current: data.current,
        remaining: data.remaining
    };

    document.getElementById('battery-voltage').textContent = `${data.voltage.toFixed(1)} V`;

    // Update battery level indicator
    const batteryLevel = document.getElementById('battery-level');
    const batteryPercent = Math.min(Math.max((data.voltage - 10) / 4 * 100, 0), 100); // Simple 10-14V scale
    batteryLevel.style.width = `${batteryPercent}%`;

    // Warning for low battery
    if (data.voltage < 10.5) {
        showToast(`Low battery: ${data.voltage.toFixed(1)}V`, 'warning');
        addChatMessage(`Warning: Low battery (${data.voltage.toFixed(1)}V)`, 'bot');
    }
});

socket.on('gps_update', (data) => {
    state.gps = {
        status: data.status,
        satellites: data.satellites,
        hdop: data.hdop
    };

    const gpsStatus = document.getElementById('gps-status');
    gpsStatus.textContent = data.status >= 3 ?
        `3D FIX (${data.satellites})` : `NO FIX (${data.satellites})`;
    gpsStatus.className = data.status >= 3 ?
        'text-xl font-bold text-green-600' : 'text-xl font-bold text-red-600';
});

socket.on('preflight_results', (data) => {
    const container = document.getElementById('preflight-results');
    container.innerHTML = '';

    if (data.error) {
        container.innerHTML = `<div class="text-red-500 p-2">${data.error}</div>`;
        addChatMessage(`Pre-flight check failed: ${data.error}`, 'bot');
        return;
    }

    const checks = {
        gps: { name: 'GPS Lock', good: '3D Fix', bad: 'No Fix', extra: data.gps.satellites ? `${data.gps.satellites} sats` : '' },
        battery: { name: 'Battery', good: 'Voltage OK', bad: 'Low Voltage', extra: data.battery.voltage ? `${data.battery.voltage.toFixed(1)}V` : '' },
        compass: { name: 'Compass', good: 'OK', bad: 'Needs Calib', extra: data.compass.message },
        arming_check: { name: 'Arming Checks', good: 'Passed', bad: 'Failed', extra: data.arming_check.message }
    };

    for (const [key, check] of Object.entries(checks)) {
        if (!data[key]) continue;

        const isOk = data[key].status;
        const div = document.createElement('div');
        div.className = `flex items-center p-2 rounded ${isOk ? 'bg-green-50' : 'bg-red-50'}`;
        div.innerHTML = `
    <span class="flex-1 font-medium">${check.name}</span>
    <span class="${isOk ? 'text-green-600' : 'text-red-600'}">
        ${isOk ? check.good : (data[key].error || check.bad)}
    </span>
    ${check.extra ? `<span class="ml-2 text-sm text-gray-500">${check.extra}</span>` : ''}
`;
        container.appendChild(div);
    }

    const overall = document.createElement('div');
    overall.className = `mt-3 p-2 rounded font-bold text-center 
${data.overall_status ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'}`;
    overall.textContent = data.overall_status ?
        '✓ ALL SYSTEMS GO' : '✗ PRE-FLIGHT CHECKS FAILED';
    container.appendChild(overall);

    addChatMessage(`Pre-flight checks ${data.overall_status ? 'passed' : 'failed'}`, 'bot');
});

socket.on('ai_response', (data) => {
    addChatMessage(data.response, 'bot');
    if (data.command) {
        addChatMessage(`Executing: ${data.command}`, 'bot');
        updateCommandHistory(data.command);
    }
});

socket.on('command_ack', (data) => {
    showToast(`${data.command.toUpperCase()} command successful`, 'success');
    addChatMessage(`Command successful: ${data.command}`, 'bot');
});

socket.on('command_error', (data) => {
    showToast(`Command failed: ${data.message}`, 'error');
    addChatMessage(`Command failed: ${data.message}`, 'bot');
});

// UI Functions
function updateConnectionStatus(connected, details = {}) {
    state.connected = connected;
    const status = document.getElementById('connection-status');
    const connectionBtn = document.getElementById('connection-btn');

    if (connected) {
        status.textContent = 'CONNECTED';
        status.classList.remove('bg-red-500', 'bg-yellow-500');
        status.classList.add('bg-green-500');
        connectionBtn.textContent = 'Disconnect';
        connectionBtn.classList.remove('bg-green-500', 'hover:bg-green-600');
        connectionBtn.classList.add('bg-gray-500', 'hover:bg-gray-600');
    } else {
        status.textContent = 'DISCONNECTED';
        status.classList.remove('bg-green-500', 'bg-yellow-500');
        status.classList.add('bg-red-500');
        connectionBtn.textContent = 'Connect';
        connectionBtn.classList.remove('bg-gray-500', 'hover:bg-gray-600');
        connectionBtn.classList.add('bg-green-500', 'hover:bg-green-600');
    }

    // Update button states
    document.getElementById('arm-btn').disabled = !connected;
    document.getElementById('disarm-btn').disabled = !connected;
    document.getElementById('takeoff-btn').disabled = !connected;
    document.getElementById('land-btn').disabled = !connected;
    document.getElementById('rtl-btn').disabled = !connected;
    document.getElementById('emergency-btn').disabled = !connected;
    document.getElementById('set-mode-btn').disabled = !connected;
    document.getElementById('preflight-btn').disabled = !connected;
}

function sendCommand(command, params = {}) {
    if (!state.connected) {
        showToast('Not connected to vehicle', 'error');
        return false;
    }

    socket.emit(command, params);
    addChatMessage(`Sent command: ${command} ${JSON.stringify(params)}`, 'user');
    updateCommandHistory(command);
    return true;
}

function processNaturalLanguageCommand(text) {
    if (!state.connected) {
        addChatMessage("Cannot process command - not connected to vehicle", 'bot');
        return;
    }

    if (!text.trim()) {
        showToast('Please enter a command', 'warning');
        return;
    }

    socket.emit('process_command', { text });
    addChatMessage(`Processing: "${text}"`, 'user');
    updateCommandHistory(text);
}

// Event listeners
document.getElementById('connection-btn').addEventListener('click', () => {
    const qgcAddress = document.getElementById('qgc-address').value.trim();
    const targetId = document.getElementById('target-id').value.trim() || '1';

    if (!state.connected) {
        // Connect logic
        if (!qgcAddress) {
            showToast('Please enter QGC address', 'warning');
            return;
        }

        document.getElementById('connection-status').textContent = 'CONNECTING';
        document.getElementById('connection-status').classList.remove('bg-red-500');
        document.getElementById('connection-status').classList.add('bg-yellow-500');

        socket.emit('connect_to_vehicle', {
            qgc_address: qgcAddress,
            target_system: parseInt(targetId)
        });

        addChatMessage(`Attempting connection to ${qgcAddress} (Target ID: ${targetId})`, 'user');
    } else {
        // Disconnect logic
        socket.emit('disconnect_from_vehicle');
        addChatMessage('Disconnecting from vehicle...', 'user');
    }
});

document.getElementById('send-button').addEventListener('click', () => {
    const message = document.getElementById('message-input').value.trim();
    document.getElementById('message-input').value = '';
    processNaturalLanguageCommand(message);
});

document.getElementById('message-input').addEventListener('keydown', (e) => {
    // Handle Enter key
    if (e.key === 'Enter') {
        const message = document.getElementById('message-input').value.trim();
        document.getElementById('message-input').value = '';
        processNaturalLanguageCommand(message);
    }

    // Handle up arrow for command history
    else if (e.key === 'ArrowUp') {
        if (state.commandHistory.length === 0) return;

        if (state.historyIndex < state.commandHistory.length - 1) {
            state.historyIndex++;
            document.getElementById('message-input').value =
                state.commandHistory[state.commandHistory.length - 1 - state.historyIndex];
        }
        e.preventDefault();
    }

    // Handle down arrow for command history
    else if (e.key === 'ArrowDown') {
        if (state.historyIndex > 0) {
            state.historyIndex--;
            document.getElementById('message-input').value =
                state.commandHistory[state.commandHistory.length - 1 - state.historyIndex];
        } else if (state.historyIndex === 0) {
            state.historyIndex = -1;
            document.getElementById('message-input').value = '';
        }
        e.preventDefault();
    }
});

document.getElementById('speech-button').addEventListener('click', () => {
    if ('webkitSpeechRecognition' in window) {
        const recognition = new webkitSpeechRecognition();

        // Configure recognition settings
        recognition.continuous = false;
        recognition.interimResults = false;
        // recognition.lang = 'en'; // Set your preferred language

        // Visual feedback
        const speechButton = document.getElementById('speech-button');
        speechButton.classList.remove('bg-purple-500');
        speechButton.classList.add('bg-purple-700', 'animate-pulse');
        showToast('Listening... Speak now', 'info');

        // Set timeout to prevent infinite listening
        const timeout = setTimeout(() => {
            recognition.stop();
            showToast('Listening timed out', 'warning');
        }, 5000); // 5 second timeout

        recognition.onresult = (event) => {
            clearTimeout(timeout);
            const transcript = event.results[0][0].transcript;
            document.getElementById('message-input').value = transcript;
            processNaturalLanguageCommand(transcript);
        };

        recognition.onerror = (event) => {
            clearTimeout(timeout);
            let errorMessage = 'Speech recognition error';

            switch (event.error) {
                case 'no-speech':
                    errorMessage = 'No speech detected. Please try again.';
                    break;
                case 'audio-capture':
                    errorMessage = 'Microphone not available';
                    break;
                case 'not-allowed':
                    errorMessage = 'Microphone access denied';
                    break;
            }

            showToast(errorMessage, 'error');
        };

        recognition.onend = () => {
            clearTimeout(timeout);
            speechButton.classList.remove('bg-purple-700', 'animate-pulse');
            speechButton.classList.add('bg-purple-500');
        };

        recognition.start();
    } else {
        showToast('Speech recognition not supported in your browser', 'warning');
    }
});


document.getElementById('arm-btn').addEventListener('click', () => {
    if (confirm('Are you sure you want to ARM the vehicle?')) {
        sendCommand('arm');
    }
});
document.getElementById('disarm-btn').addEventListener('click', () => {
    if (confirm('Are you sure you want to DISARM the vehicle?')) {
        sendCommand('disarm');
    }
});

document.getElementById('takeoff-btn').addEventListener('click', () => {
    const alt = prompt('Enter takeoff altitude (meters):', '10');
    if (alt && !isNaN(alt)) {
        sendCommand('takeoff', { altitude: parseFloat(alt) });
    }
});

document.getElementById('land-btn').addEventListener('click', () => {
    if (confirm('Initiate landing sequence?')) {
        sendCommand('land');
    }
});

document.getElementById('rtl-btn').addEventListener('click', () => {
    if (confirm('Return to launch location?')) {
        sendCommand('rtl');
    }
});

document.getElementById('emergency-btn').addEventListener('click', () => {
    if (confirm('EMERGENCY: This will kill motors immediately! Proceed?')) {
        sendCommand('emergency_stop');
    }
});

document.getElementById('set-mode-btn').addEventListener('click', () => {
    const mode = document.getElementById('flight-mode-select').value;

    // Validate mode before sending
    if (!mode) {
        showToast('Please select a valid flight mode', 'warning');
        return;
    }

    sendCommand('set_mode', { mode });
});

document.getElementById('preflight-btn').addEventListener('click', () => {
    socket.emit('run_preflight');
    addChatMessage('Running pre-flight checks...', 'user');
});

// Initialize
updateConnectionStatus(false);
updateAttitudeIndicator(0, 0);