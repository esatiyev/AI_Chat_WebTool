// Map Initialization
const map = L.map('map').setView([0, 0], 15);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
}).addTo(map);

const droneIcon = L.divIcon({
    className: 'drone-marker',
    html: '<div class="drone-arrow"></div>',
    iconSize: [24, 24],
    iconAnchor: [12, 12]
});

const droneMarker = L.marker([0, 0], {icon: droneIcon}).addTo(map);

// Attitude Indicator Animation
function updateAttitudeIndicator(roll, pitch) {
    const maxTilt = 30; // degrees
    const ball = document.getElementById('attitude-ball');
    
    // Normalize pitch to -1 (max down) to 1 (max up)
    const normalizedPitch = Math.min(Math.max(pitch / maxTilt, -1), 1);
    // Normalize roll to -1 (left) to 1 (right)
    const normalizedRoll = Math.min(Math.max(roll / maxTilt, -1), 1);
    
    ball.style.transform = `translate(-50%, -50%) 
        translateX(${normalizedRoll * 30}px) 
        translateY(${normalizedPitch * 30}px) 
        rotate(${roll}deg)`;
}

// Socket Handlers
socket.on('position_update', (data) => {
    // Update map
    const newPos = [data.lat, data.lon];
    droneMarker.setLatLng(newPos);
    if (!map.getBounds().contains(newPos)) {
        map.setView(newPos, map.getZoom());
    }
    
    // Update position displays
    document.getElementById('lat-value').textContent = data.lat.toFixed(6);
    document.getElementById('lon-value').textContent = data.lon.toFixed(6);
    document.getElementById('alt-value').textContent = `${data.alt.toFixed(1)} m`;
});

socket.on('attitude_update', (data) => {
    updateAttitudeIndicator(data.roll, data.pitch);
    document.getElementById('roll-value').textContent = `${data.roll.toFixed(1)}°`;
    document.getElementById('pitch-value').textContent = `${data.pitch.toFixed(1)}°`;
    document.getElementById('yaw-value').textContent = `${data.yaw.toFixed(1)}°`;
});

// Add CSS for drone marker
const style = document.createElement('style');
style.textContent = `
.drone-marker .drone-arrow {
    width: 0; 
    height: 0; 
    border-left: 12px solid transparent;
    border-right: 12px solid transparent;
    border-bottom: 24px solid #e74c3c;
    position: relative;
}
.drone-marker .drone-arrow:after {
    content: "";
    position: absolute;
    left: -6px;
    top: 6px;
    width: 12px;
    height: 12px;
    background-color: #3498db;
    border-radius: 50%;
}
`;
document.head.appendChild(style);

// Pre-flight check handling
document.getElementById('run-preflight').addEventListener('click', () => {
    socket.emit('run_preflight');
});

socket.on('preflight_results', (data) => {
    const container = document.getElementById('preflight-results');
    container.innerHTML = '';
    
    if (data.error) {
        container.innerHTML = `<div class="text-red-500">${data.error}</div>`;
        return;
    }
    
    const checkItems = {
        gps: { name: 'GPS Lock', good: '3D Fix', bad: 'No Fix' },
        battery: { name: 'Battery', good: 'Voltage OK', bad: 'Low Voltage' },
        compass: { name: 'Compass', good: 'Calibrated', bad: 'Needs Calibration' },
        arming_check: { name: 'Arming Checks', good: 'Passed', bad: 'Failed' }
    };
    
    for (const [key, check] of Object.entries(data)) {
        if (key === 'overall_status' || key === 'error') continue;
        
        const item = checkItems[key];
        const isOk = check.status;
        
        const div = document.createElement('div');
        div.className = `flex items-center p-2 rounded ${isOk ? 'bg-green-50' : 'bg-red-50'}`;
        div.innerHTML = `
            <span class="flex-1">${item.name}</span>
            <span class="${isOk ? 'text-green-600' : 'text-red-600'} font-medium">
                ${isOk ? item.good : (check.error || item.bad)}
            </span>
            ${check.satellites ? `<span class="ml-2 text-sm">${check.satellites} sats</span>` : ''}
            ${check.voltage ? `<span class="ml-2 text-sm">${check.voltage.toFixed(1)}V</span>` : ''}
        `;
        container.appendChild(div);
    }
    
    const overall = document.createElement('div');
    overall.className = `mt-3 p-2 rounded font-bold text-center 
        ${data.overall_status ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'}`;
    overall.textContent = data.overall_status ? 
        '✓ ALL SYSTEMS GO' : '✗ PRE-FLIGHT CHECKS FAILED';
    container.appendChild(overall);
});