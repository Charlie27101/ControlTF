import serial
import time
import struct
import csv
import struct
import math

class RMDX6:
    def __init__(self, motor_id, com_port, baudrate=115200, tsamp=0.02):
        # Initialize motor parameters and open the serial port
        self.id = motor_id                 # Motor ID
        self.gearatio = 8                  # Gear ratio for speed conversion
        self.delay = tsamp                 # Delay after command to allow response
        self.port = serial.Serial(com_port, baudrate=baudrate, timeout=1)  # Open serial port

    def send485(self, scmd, data_bytes):
        # Send command to motor over RCOMS-485 using specified protocol
        header = [0x3E, self.id, 8]  # Start byte, motor ID, fixed data length (8 bytes)
        cmd = [int(scmd[i:i+2], 16) for i in (0, 2)] if len(scmd) == 4 else [int(scmd, 16), 0]  # Parse command bytes
        msg = bytearray(header + cmd + data_bytes)  # Full message before CRC
        crc = self.get_crc16(msg)  # Calculate CRC16
        msg += struct.pack('<H', crc)  # Append CRC to message
        self.port.write(msg)  # Send message
        time.sleep(self.delay)  # Wait for response
        response = self.port.read(13)  # Read 13-byte response
        return response

    def send_rms(self, command_id, data_bytes):
        # Send a more generic RMS-formatted command
        start = 0x3E  # Start byte
        length = len(data_bytes) + 2  # Length includes command byte and data
        header = [start, self.id, length, command_id]  # Header with command
        msg = bytearray(header + data_bytes)  # Construct full message
        crc = self.get_crc16(msg)  # CRC16
        msg += struct.pack('<H', crc)  # Append CRC
        self.port.write(msg)  # Send
        time.sleep(self.delay)  # Wait
        response = self.port.read(self.port.in_waiting or 1)  # Read available response
        return response

    def get_crc16(self, data):
        # Calculate CRC16 using Modbus standard
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def get_angle(self):
        # Request multi-turn angle from motor
        response = self.send485('92', [0]*6)
        if len(response) >= 11:
            angle_raw = struct.unpack('<i', response[7:11])[0]  # Extract signed 32-bit int
            return angle_raw / 100.0  # Convert to degrees
        else :
            print("Error: Invalid response length for angle request.")
            return None
        return None

    def get_rpm(self):
        # Request motor speed in RPM
        response = self.send485('9C', [0]*6)
        if len(response) >= 9:
            vel_raw = struct.unpack('<h', response[7:9])[0]  # Extract signed 16-bit int
            return vel_raw
        return None

    def get_state(self):
        # Read temperature, current, velocity and angle from motor
        response = self.send485('9C', [0]*6)
        if len(response) >= 11:
            temp = response[4]  # Temperature in Celsius
            current = struct.unpack('<h', response[5:7])[0] / 100.0  # Current in A
            vel = struct.unpack('<h', response[7:9])[0] * self.gearatio  # Speed scaled, the spped returned is in dps
            angle = struct.unpack('<h', response[9:11])[0]  # Angle in degrees
            return temp, current, vel, angle
        return None, None, None, None

    def get_state_current(self):
        # Read temperature, current, velocity and angle from motor
        response = self.send485('9C', [0]*6)
        if len(response) >= 11:
            current = struct.unpack('<h', response[5:7])[0] / 100.0  # Current in A
            return current
        else:
            return None

    def get_angle(self):
        # Read temperature, current, velocity and angle from motor
        response = self.send485('9C', [0]*6)
        if len(response) >= 11:
            angle = struct.unpack('<h', response[9:11])[0]  # Angle in degrees
            return angle
        return None

    def get_state_raw(self):
        # Read 13 byte array from motor for temp, current, velocity and angle
        response = self.send485('9C', [0]*6)
        if len(response) >= 11:
            return response
        else:
            return None


    def set_torque(self, current):
        # Send torque command to motor (current in A)
        current_val = int(current * 100)  # Convert to centiamps
        current_bytes = struct.pack('<h', current_val)  # Pack as little-endian
        data_bytes = [0, 0] + list(current_bytes) + [0, 0]  # Pad to 6 bytes
        response = self.send485('A1', data_bytes)  # Send command
        if len(response) >= 11:
            temp = response[4]
            curr = struct.unpack('<h', response[5:7])[0] / 100.0
            vel = struct.unpack('<h', response[7:9])[0] * self.gearatio
            angle = struct.unpack('<h', response[9:11])[0]
            return temp, curr, vel, angle
        return None, None, None, None
    
    def quick_set_torque(self, current):
        # Send torque command to motor (current in A)
        current_val = int(current * 100)  # Convert to centiamps
        current_bytes = struct.pack('<h', current_val)  # Pack as little-endian
        data_bytes = [0, 0] + list(current_bytes) + [0, 0]  # Pad to 6 bytes
        response = self.send485('A1', data_bytes)  # Send command
        if len(response) >= 11:
            temp = response[4]
            curr = struct.unpack('<h', response[5:7])[0] / 100.0
            vel = struct.unpack('<h', response[7:9])[0] * self.gearatio
            angle = struct.unpack('<h', response[9:11])[0]
            return temp, curr, vel, angle
        return None, None, None, None

    def set_speed_rpm(self, speed):
    # Send speed in rpm to motor
        #speedTodps = int(sped / 6)  # Convert to dps 
        current_bytes = struct.pack('<h', int(speed * 6))  # Pack as little-endian
        data_bytes = [0, 0, 0] + list(current_bytes) + [0] # last 4 bytes are speed
        response = self.send485('A2', data_bytes) # Send command
        if len(response) >= 11:
            return response
        return None

    def set_position(self, angle_deg, max_vel=1000):
        # Command motor to move to specific angle
        angle_bytes = struct.pack('<l', int(angle_deg * 100*8))  # Pack angle (deg * 100)
        vel_bytes = struct.pack('<h', int(max_vel))  # Pack velocity (deg * 100)
        data_bytes = [0] +list(vel_bytes) +list(angle_bytes)  # Convert to list for sending
        response = self.send485('A4', data_bytes) # Send command
        if len(response) >= 11:
            return response
        return None

    def set_speed_dps(self, speed):
    # Send speed in dps to motor
        current_bytes = struct.pack('<h', speed)  # Pack as little-endian
        data_bytes = [0, 0, 0] + list(current_bytes) + [0] # last 4 bytes are speed
        response = self.send485('A2', data_bytes)  # Send command
        if len(response) >= 11:
            temp = response[4]
            curr = struct.unpack('<h', response[5:7])[0] / 100.0
            vel = struct.unpack('<h', response[7:9])[0] * self.gearatio
            angle = struct.unpack('<h', response[9:11])[0]
            return temp, curr, vel, angle
        return None, None, None, None

    def move_to_position(self, angle_deg, max_vel):
        # Command motor to move to specific angle at max velocity
        vel_bytes = struct.pack('<h', int(max_vel))  # Pack max velocity (dps)
        angle_bytes = struct.pack('<i', int(angle_deg * 100 ))  # Pack angle (deg * 100)
        data_bytes = list(vel_bytes + angle_bytes)  # Combine to 6 bytes
        return self.send485('A4', data_bytes)  # Send command

    def stop(self):
        # Stop the motor
        return self.send485('81', [0]*6)

    def shutdown(self):
        # Shutdown motor power
        return self.send485('80', [0]*6)

    def close(self):
        # Safely shutdown and close serial port
        self.shutdown()
        self.port.close()

#csv operations
def csv_from_motor_data(filename, data):
        # Create CSV and write headers
    with open(filename+'.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Time', 'Temperature', 'Current', 'Velocity', 'Angle'])  # Header row
        for response, timestamp in motorData:
            try:
                temp = response[4]
                curr = struct.unpack('<h', response[5:7])[0] / 100.0
                vel = struct.unpack('<h', response[7:9])[0] * motor.gearatio
                angle = struct.unpack('<h', response[9:11])[0]
                writer.writerow([timestamp, temp, curr, vel, angle])
            except Exception as e:
                print(f"Error parsing response at {timestamp:.3f}s: {e}")

def step_entry(t,stepStart, stepend=False):
    # create a steps function
    if not stepend:
        return 1 if t >= stepStart else 0
    else:
        if t < 0:
            return 0
        return 1+step_entry(t - stepStart, stepStart, True)
    
class pid_controller:
    def __init__(self, Kp=1, Ki=1, Kd=1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.prev_error = 0

    def get_pid(self):
        return self.Kp, self.Ki, self.Kd, self.integral, self.prev_error

    def control(self, setpoint, dt, measured):
        # Simple PID controller
        error = setpoint - measured
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output

# Example usage: log values for 10 seconds
if __name__ == '__main__':
    motor = RMDX6(motor_id=1, com_port='COM3', tsamp=0.02)  # Adjust COM port as needed
    pid = pid_controller(Kp=0.1, Ki=0.01, Kd=0.005)
    duration = 10  # Duration to log data in seconds
    motorData = []  # List to store (response, timestamp)
    start_time = time.time()
    try:
        while (time.time() - start_time) < duration:
            timestamp = time.time() - start_time
            response = motor.get_state_raw()
            motorData.append((response, timestamp))
            pid.control(setpoint=0.5, dt=timestamp, measured=struct.unpack('<h', response[5:7])[0] / 100.0)  # Current in A)

    except KeyboardInterrupt:
        print("Logging interrupted by user.")
    finally:
        motor.close()
        date = time.strftime("%Y%m%d-%H%M%S")
        csv_from_motor_data('motor_log'+date, motorData)
        print("Data logged to motor_log.csv")