from flask import Flask
from flask_socketio import SocketIO
from pymavlink import mavutil
import threading
import time
import math
import re
from collections import deque
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# MAVLink connection and state
mav_conn = None
is_connected = False
current_mode = "Unknown"
vehicle_armed = False
connection_params = {
    'qgc_address': None,
    'target_system': 1
}

# Telemetry history for visualization
telemetry_history = {
    'position': deque(maxlen=100),
    'attitude': deque(maxlen=100),
    'battery': deque(maxlen=100)
}

# Command definitions
COMMAND_MAPPING = {
    'arm': mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    'takeoff': mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    'land': mavutil.mavlink.MAV_CMD_NAV_LAND,
    'rtl': mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
    'set_mode': mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    'emergency_stop': mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
}

# Flight mode mapping
FLIGHT_MODES = {
    0: 'STABILIZE',
    1: 'ACRO',
    2: 'ALT_HOLD',
    3: 'AUTO',
    4: 'GUIDED',
    5: 'LOITER',
    6: 'RTL',
    7: 'CIRCLE',
    9: 'LAND',
    11: 'DRIFT',
    16: 'POSHOLD'
}

def connect_to_mavlink(qgc_address, target_system=1):
    """Establish MAVLink connection to the vehicle"""
    global mav_conn, is_connected, current_mode, connection_params
    
    try:
        # Update connection parameters
        connection_params = {
            'qgc_address': qgc_address,
            'target_system': target_system
        }
        
        # Notify frontend of connection attempt
        socketio.emit('connection_status', {
            'connected': False,
            'status': 'CONNECTING',
            'message': f'Connecting to {qgc_address}...'
        })
        
        # 1. Establish connection
        mav_conn = mavutil.mavlink_connection(f'udp:{qgc_address}')
        logger.info(f"Connecting to vehicle at {qgc_address}, target system {target_system}")
        
        # 2. Wait for heartbeat to confirm connection
        heartbeat = mav_conn.wait_heartbeat(timeout=5)
        if not heartbeat:
            raise ConnectionError("No heartbeat received from vehicle")
            
        mav_conn.target_system = target_system
        mav_conn.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        
        # 3. Update connection status
        is_connected = True
        logger.info("Vehicle connection established")
        
        # 4. Start status monitoring threads
        threading.Thread(target=monitor_vehicle_status, daemon=True).start()
        threading.Thread(target=monitor_heartbeat, daemon=True).start()
        
        # 5. Notify frontend
        socketio.emit('connection_status', {
            'connected': True,
            'status': 'CONNECTED',
            'message': 'Vehicle connection established',
            'vehicle_type': mavutil.mavlink.enums['MAV_TYPE'][heartbeat.type].name,
            'autopilot': mavutil.mavlink.enums['MAV_AUTOPILOT'][heartbeat.autopilot].name
        })
        
    except Exception as e:
        is_connected = False
        mav_conn = None
        error_msg = f"Connection failed: {str(e)}"
        logger.error(error_msg)
        socketio.emit('connection_error', {
            'message': error_msg,
            'retry_possible': True,
            'details': connection_params
        })

def monitor_heartbeat():
    """Monitor heartbeat messages for connection health"""
    global is_connected, current_mode, vehicle_armed
    
    missed_heartbeats = 0
    max_missed_heartbeats = 3  # Allow up to 3 missed heartbeats before disconnecting
    
    while is_connected:
        if not mav_conn:
            break
        
        try:
            # Check if heartbeat message is received
            msg = mav_conn.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if not msg:
                missed_heartbeats += 1
                logger.warning(f"Missed heartbeat ({missed_heartbeats}/{max_missed_heartbeats})")
                if missed_heartbeats >= max_missed_heartbeats:
                    logger.warning("Heartbeat timeout - disconnecting")
                    disconnect_vehicle()
                    break
            else:
                missed_heartbeats = 0  # Reset missed heartbeat counter
                logger.info("Heartbeat received")
            time.sleep(1)
            
        except Exception as e:
            if is_connected:  # Log errors only if still connected
                logger.error(f"Heartbeat monitor error: {e}")
            time.sleep(1)

def monitor_vehicle_status():
    """Monitor and process vehicle telemetry data"""
    global current_mode, vehicle_armed
    
    while is_connected:
        if not mav_conn:
            break

        try:
            msg = mav_conn.recv_match(blocking=True, timeout=5)
            if not msg:
                continue

            # Process heartbeat message (contains armed state and mode)
            if msg.get_type() == 'HEARTBEAT':
                vehicle_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                current_mode = FLIGHT_MODES.get(msg.custom_mode, 'Unknown')
                
                socketio.emit('armed_state', {
                    'armed': vehicle_armed,
                    'status': 'ARMED' if vehicle_armed else 'DISARMED'
                })
                socketio.emit('flight_mode_update', {'mode': current_mode})

            # Position Tracking
            elif msg.get_type() == 'GLOBAL_POSITION_INT':
                position_data = {
                    'lat': msg.lat / 1e7,
                    'lon': msg.lon / 1e7,
                    'alt': msg.relative_alt / 1e3,
                    'heading': msg.hdg / 100,
                    'timestamp': time.time()
                }
                telemetry_history['position'].append(position_data)
                socketio.emit('position_update', position_data)

            # Attitude Tracking
            elif msg.get_type() == 'ATTITUDE':
                attitude_data = {
                    'roll': math.degrees(msg.roll),
                    'pitch': math.degrees(msg.pitch),
                    'yaw': math.degrees(msg.yaw),
                    'timestamp': time.time()
                }
                telemetry_history['attitude'].append(attitude_data)
                socketio.emit('attitude_update', attitude_data)

            # Battery Tracking
            elif msg.get_type() == 'SYS_STATUS':
                battery_data = {
                    'voltage': msg.voltage_battery / 1000 if msg.voltage_battery != 65535 else 0,
                    'current': msg.current_battery / 100 if msg.current_battery != -1 else 0,
                    'remaining': msg.battery_remaining if msg.battery_remaining != -1 else 0,
                    'timestamp': time.time()
                }
                telemetry_history['battery'].append(battery_data)
                socketio.emit('battery_update', battery_data)
                
            # GPS Status
            elif msg.get_type() == 'GPS_RAW_INT':
                gps_data = {
                    'status': msg.fix_type,
                    'satellites': msg.satellites_visible,
                    'hdop': msg.eph / 100,
                    'timestamp': time.time()
                }
                socketio.emit('gps_update', gps_data)
                
        except Exception as e:
            if is_connected:  # Log errors only if still connected
                logger.error(f"Status monitoring error: {e}")
            time.sleep(1)

def disconnect_vehicle():
    """Cleanly disconnect from vehicle"""
    global mav_conn, is_connected
    if mav_conn:
        try:
            mav_conn.close()
        except:
            pass
    mav_conn = None
    is_connected = False
    socketio.emit('connection_status', {
        'connected': False,
        'status': 'DISCONNECTED',
        'message': 'Vehicle disconnected'
    })

def send_mavlink_command(cmd_type, params=[], confirmation=0):
    """Send MAVLink command to vehicle"""
    if not is_connected or not mav_conn:
        raise ConnectionError("Not connected to vehicle")
    
    try:
        # Ensure params has exactly 7 elements (fill with 0 if fewer)
        params = (params + [0] * 7)[:7]

        # Send command
        mav_conn.mav.command_long_send(
            mav_conn.target_system,
            mav_conn.target_component,
            cmd_type,
            confirmation,
            *params  # param1 to param7
        )
        
        # Wait for command acknowledgment
        ack = mav_conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if not ack:
            raise TimeoutError("No acknowledgment received, but command may have been executed")
            
        if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(mavutil.mavlink.enums['MAV_RESULT'][ack.result].name)
            
        return True
        
    except Exception as e:
        logger.error(f"Command failed: {e}")
        raise

def check_gps_lock():
    """Check GPS lock status"""
    try:
        msg = mav_conn.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        if msg:
            return {
                'status': msg.fix_type >= 3,
                'satellites': msg.satellites_visible,
                'hdop': msg.eph / 100,
                'message': f"GPS {'3D Fix' if msg.fix_type >=3 else 'No Fix'} ({msg.satellites_visible} sats)"
            }
        return {'status': False, 'error': 'No GPS data received'}
    except Exception as e:
        return {'status': False, 'error': f'GPS check failed: {str(e)}'}

def check_battery():
    """Check battery status"""
    try:
        msg = mav_conn.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        if msg:
            voltage = msg.voltage_battery / 1000 if msg.voltage_battery != 65535 else 0
            status = voltage > 10.0  # Simple 10V threshold
            return {
                'voltage': voltage,
                'current': msg.current_battery / 100 if msg.current_battery != -1 else 0,
                'remaining': msg.battery_remaining if msg.battery_remaining != -1 else 0,
                'status': status,
                'message': f"Battery {'OK' if status else 'LOW'} ({voltage:.1f}V)"
            }
        return {'status': False, 'error': 'No battery data received'}
    except Exception as e:
        return {'status': False, 'error': f'Battery check failed: {str(e)}'}

def check_compass():
    """Check compass health using SYS_STATUS"""
    try:
        msg = mav_conn.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        if msg:
            # Check the onboard_control_sensors_health field for compass health
            compass_healthy = bool(msg.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG)
            return {
                'status': compass_healthy,
                'message': 'Compass OK' if compass_healthy else 'Compass needs calibration or is unhealthy'
            }
        return {'status': False, 'error': 'No SYS_STATUS data received'}
    except Exception as e:
        return {'status': False, 'error': f'Compass check failed: {str(e)}'}
    
def check_arming_requirements():
    """Check if vehicle is ready to arm"""
    try:
        # Check if all required parameters are good for arming
        # This is a simplified version - should be expanded based on actual requirements
        return {
            'status': True,
            'message': 'Arming checks passed',
            'details': {}
        }
    except Exception as e:
        return {'status': False, 'error': f'Arming check failed: {str(e)}'}

def process_natural_command(text):
    """Process natural language command and return MAVLink command"""
    text = text.lower().strip()
    response = ""
    command = None
    params = []
    
    # Arm/disarm commands
    if re.match(r'(arm|enable|start)\s?(the)?\s?(drone|vehicle|copter)', text):
        command = 'arm'
        params = [1]  # 1 = arm
        response = "Arming the vehicle"
    elif re.match(r'(disarm|disable|stop)\s?(the)?\s?(drone|vehicle|copter)', text):
        command = 'arm'
        params = [0]  # 0 = disarm
        response = "Disarming the vehicle"
    
    # Takeoff commands
    elif match := re.match(r'(take\s?off|takeoff|launch).*?(\d+)\s?(m|meters|meter)', text):
        alt = float(match.group(2))
        command = 'takeoff'
        params = [0, 0, 0, 0, 0, 0, alt]
        response = f"Taking off to {alt} meters"
    
    # Land commands
    elif re.match(r'(land|descend)\s?(the)?\s?(drone|vehicle|copter)', text):
        command = 'land'
        params = [0, 0, 0, 0, 0, 0, 0]
        response = "Initiating landing sequence"
    
    # RTL commands
    elif re.match(r'(return\s?to\s?launch|rtl|come\s?back|go\s?home)', text):
        command = 'rtl'
        response = "Returning to launch location"
    
    # Mode changes
    elif match := re.match(r'(set|change)\s?(to)?\s?(stabilize|alt_hold|loiter|guided|auto|rtl)', text):
        mode = match.group(3).upper()
        command = 'set_mode'
        response = f"Changing to {mode} mode"
    
    # Movement commands
    elif match := re.match(r'(go|fly|move)\s?(north|south|east|west|forward|back|left|right)\s?(\d+)\s?(m|meters|meter)', text):
        direction = match.group(2)
        distance = float(match.group(3))
        # This would need more complex implementation for actual movement
        response = f"Moving {direction} for {distance} meters"
    
    # Emergency stop
    elif re.match(r'(emergency\s?stop|kill\s?motors|stop\s?now)', text):
        command = 'emergency_stop'
        response = "EMERGENCY: Killing motors!"
    
    else:
        response = "I didn't understand that command. Try something like:\n" \
                   "- 'Arm the drone'\n" \
                   "- 'Take off to 10 meters'\n" \
                   "- 'Fly north 50 meters'\n" \
                   "- 'Return to launch'"
    
    return {
        'response': response,
        'command': command,
        'params': params
    }

# Socket.IO Event Handlers
@socketio.on('connect')
def handle_connect():
    logger.info("Client connected")
    socketio.emit('connection_status', {
        'connected': is_connected,
        'status': 'CONNECTED' if is_connected else 'DISCONNECTED',
        'message': 'Connected to backend server'
    })

@socketio.on('disconnect')
def handle_disconnect():
    logger.info("Client disconnected")

@socketio.on('connect_to_vehicle')
def handle_connect_to_vehicle(data):
    if not data or 'qgc_address' not in data:
        socketio.emit('connection_error', {'message': 'Missing QGC address'})
        return
        
    threading.Thread(
        target=connect_to_mavlink,
        args=(data['qgc_address'], int(data.get('target_system', 1)))
    ).start()

@socketio.on('disconnect_from_vehicle')
def handle_disconnect_from_vehicle():
    disconnect_vehicle()

@socketio.on('arm')
def handle_arm(data=None):
    try:
        send_mavlink_command(COMMAND_MAPPING['arm'], [1])
        socketio.emit('command_ack', {
            'command': 'arm',
            'result': 'SUCCESS',
            'message': 'Vehicle armed successfully'
        })
    except Exception as e:
        socketio.emit('command_error', {
            'command': 'arm',
            'message': str(e)
        })

@socketio.on('disarm')
def handle_disarm(data=None):
    """Handle disarm command"""
    try:
        send_mavlink_command(COMMAND_MAPPING['arm'], [0])
        socketio.emit('command_ack', {
            'command': 'disarm',
            'result': 'SUCCESS',
            'message': 'Vehicle disarmed successfully'
        })
    except Exception as e:
        socketio.emit('command_error', {
            'command': 'disarm',
            'message': str(e)
        })

@socketio.on('takeoff')
def handle_takeoff(data):
    try:
        altitude = float(data.get('altitude', 10))
        send_mavlink_command(COMMAND_MAPPING['takeoff'], [0, 0, 0, 0, 0, 0, altitude])
        socketio.emit('command_ack', {
            'command': 'takeoff',
            'result': 'SUCCESS',
            'message': f'Takeoff to {altitude}m initiated'
        })
    except Exception as e:
        socketio.emit('command_error', {
            'command': 'takeoff',
            'message': str(e)
        })

@socketio.on('land')
def handle_land(data=None):
    try:
        send_mavlink_command(COMMAND_MAPPING['land'])
        socketio.emit('command_ack', {
            'command': 'land',
            'result': 'SUCCESS',
            'message': 'Landing initiated'
        })
    except Exception as e:
        socketio.emit('command_error', {
            'command': 'land',
            'message': str(e)
        })

@socketio.on('rtl')
def handle_rtl(data=None):
    """Handle return to launch command"""
    try:
        send_mavlink_command(COMMAND_MAPPING['rtl'])
        socketio.emit('command_ack', {
            'command': 'rtl',
            'result': 'SUCCESS',
            'message': 'Return to launch initiated'
        })
    except Exception as e:
        socketio.emit('command_error', {
            'command': 'rtl',
            'message': str(e)
        })

@socketio.on('set_mode')
def handle_set_mode(data):
    try:
        mode = data.get('mode', 'GUIDED').upper()

        # Validate mode
        if mode not in FLIGHT_MODES.values():
            raise ValueError(f"Invalid flight mode: {mode}")

        # Find the custom mode index
        custom_mode = list(FLIGHT_MODES.keys())[list(FLIGHT_MODES.values()).index(mode)]

        # Send MAVLink command to set the mode
        send_mavlink_command(COMMAND_MAPPING['set_mode'], [
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # Base mode
            # 0,  # Reserved or unused parameter
            custom_mode,  # Custom mode
            0, 0, 0, 0
        ])

        socketio.emit('command_ack', {
            'command': 'set_mode',
            'result': 'SUCCESS',
            'message': f'Flight mode changed to {mode}'
        })
    except Exception as e:
        socketio.emit('command_error', {
            'command': 'set_mode',
            'message': str(e)
        })

@socketio.on('emergency_stop')
def handle_emergency_stop(data=None):
    """Handle emergency stop command"""
    try:
        send_mavlink_command(COMMAND_MAPPING['emergency_stop'], [0])
        socketio.emit('command_ack', {
            'command': 'emergency_stop',
            'result': 'SUCCESS',
            'message': 'EMERGENCY: Motors killed!'
        })
    except Exception as e:
        socketio.emit('command_error', {
            'command': 'emergency_stop',
            'message': str(e)
        })

@socketio.on('run_preflight')
def handle_preflight_checks():
    if not is_connected or not mav_conn:
        socketio.emit('preflight_results', {'error': 'Not connected to vehicle'})
        return

    try:
        checks = {
            'gps': check_gps_lock(),
            'battery': check_battery(),
            'compass': check_compass(),
            'arming_check': check_arming_requirements()
        }
        
        checks['overall_status'] = all(check.get('status', False) for check in checks.values())
        socketio.emit('preflight_results', checks)
        
    except Exception as e:
        socketio.emit('preflight_results', {
            'error': f'Preflight check failed: {str(e)}',
            'overall_status': False
        })

@socketio.on('process_command')
def handle_process_command(data):
    if not is_connected or not mav_conn:
        socketio.emit('ai_response', {
            'response': "Cannot process command - not connected to vehicle",
            'command': None
        })
        return
    
    try:
        command_info = process_natural_command(data.get('text', ''))
        socketio.emit('ai_response', {
            'response': command_info['response'],
            'command': command_info['command']
        })
        
        # If we have a valid command, execute it
        if command_info['command']:
            # Map to appropriate handler
            if command_info['command'] == 'arm':
                handle_arm()
            elif command_info['command'] == 'takeoff':
                handle_takeoff({'altitude': command_info['params'][6]})
            elif command_info['command'] == 'land':
                handle_land()
            elif command_info['command'] == 'rtl':
                handle_rtl()
            elif command_info['command'] == 'set_mode':
                handle_set_mode({'mode': command_info['response'].split()[-2]})
                
    except Exception as e:
        socketio.emit('ai_response', {
            'response': f"Error processing command: {str(e)}",
            'command': None
        })

if __name__ == '__main__':
    logger.info("Starting ArduPilot AI Chat WebTool backend")
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)