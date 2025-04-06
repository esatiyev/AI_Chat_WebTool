# ArduPilot AI_Chat_WebTool

A web-based interface for controlling ArduPilot vehicles using natural language commands, with real-time telemetry visualization.

![Screenshot of the interface](https://github.com/user-attachments/assets/6e12e86f-2c6e-4344-8361-f308192300be)

## Features

### Core Functionality
- Natural language command processing (voice or text)
- MAVLink vehicle connection management
- Real-time telemetry monitoring
- Interactive map with drone positioning
- Attitude indicator visualization

### Command Capabilities
- Vehicle arming/disarming
- Takeoff/landing/RTL
- Flight mode changes
- Directional movement (north/south/east/west)
- Emergency stop
- Pre-flight checks

### Natural Language Command Processing

The system uses regex patterns to interpret natural language commands. Here's the complete command syntax:

#### Command Patterns

```python
# Arm/Disarm
r'(arm|enable|start)\s?(the)?\s?(drone|vehicle|copter)'
r'(disarm|disable|stop)\s?(the)?\s?(drone|vehicle|copter)'

# Takeoff 
r'(take\s?off|takeoff|launch).*?(\d+)\s?(m|meters|meter)'

# Movement
r'(go|fly|move)\s?(north|south|east|west|forward|back|left|right)\s?(\d+)\s?(m|meters|meter)'

# Mode Changes
r'(set|change)\s?(to)?\s?(stabilize|alt_hold|loiter|guided|auto|rtl)'

# Emergency
r'(emergency\s?stop|kill\s?motors|stop\s?now)'
```

### Technologies Used

**Backend:**
- Python Flask
- Flask-SocketIO
- pymavlink
- MAVLink protocol
- re

**Frontend:**
- HTML5/CSS3/JS
- Socket.IO client
- Leaflet.js (mapping)
- Web Speech API (voice commands)

## Installation

### Prerequisites
- Python 3.8+
- ArduPilot SITL or physical vehicle

### Setup
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/AI_Chat_WebTool.git
   cd AI_Chat_WebTool
   ```      
2. Install required packages
   ```bash
   pip install flask flask-socketio pymavlink python-dotenv
3. Run:
   on first tab:
   ```bash
   python backend/main.py
   ```
   open new tab on terminal
   ```bash
   python -m http.server 8000
   ```
4. Open the website on chrome:
   http://localhost:8000/webui/
5. Run ArduPilot:
   open new tab on terminal
   ```bash
   sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out 127.0.0.1:14540
6. Run QGC or MP
