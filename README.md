# ArduPilot AI_Chat_WebTool

A web-based interface for controlling ArduPilot vehicles using natural language commands (voice/text) with both AI and regex processing, plus real-time telemetry visualization.


![Screenshot of the interface](https://github.com/user-attachments/assets/6e12e86f-2c6e-4344-8361-f308192300be)

## Features

### Core Functionality
- Dual-mode command processing (Gemini AI with regex fallback)
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

#### AI-Powered Processing (Primary)

- Uses Google's Gemini AI for flexible natural language understanding
- Supports conversational commands and variations
- Falls back to regex when AI is unavailable

#### Regex Patterns (Fallback)

The system uses regex patterns to interpret natural language commands. Here's the complete command syntax:

#### Command Patterns

```python
# Arm/Disarm
r'(arm|enable|start)\s?(the)?\s?(drone|vehicle|copter)'
r'(disarm|disable|stop)\s?(the)?\s?(drone|vehicle|copter)'

# Takeoff 
r'(take\s?off|takeoff|launch).*?(\d+)\s?(m|meters|meter)'

# Movement
r'(go|fly|move)\s?(north|south|east|west)\s?(\d+)\s?(m|meters|meter)'

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
- Google Gemini AI API
- Regex fallback processing

**Frontend:**
- HTML5/CSS3/JS
- Socket.IO client
- Leaflet.js (mapping)
- Web Speech API (voice commands)

### Key Improvements:
- Added rate limiting (55 requests/minute) for Gemini API
- Enhanced error handling for AI responses
- Automatic parameter validation and conversion
- Seamless fallback to regex when AI is unavailable


## Installation

### Prerequisites
- Python 3.8+
- ArduPilot SITL or physical vehicle
- Gemini API Key: You can get one from here: https://aistudio.google.com/apikey

### Setup
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/AI_Chat_WebTool.git
   cd AI_Chat_WebTool
   ```      
2. Install required packages
   ```bash
   pip install flask flask-socketio pymavlink python-dotenv
   pip install -q -U google-generativeai
3. Modify .env file:
   1. Open backend/.env file 
   2. Replace GEMINI_API_KEY("YOUR_GEMINI_API_KEY") with actual key you got
   3. Change SECRET_KEY to anything you want
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
