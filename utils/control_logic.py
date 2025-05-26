
import serial
import serial.tools.list_ports
import time
START_BYTE = 254
END_BYTE = 255
MAX_VIBE = 253
SERVO_MIN = 0
SERVO_MAX = 180

class HapticCommander:
    def __init__(self, serial_port, baud):
        
        self.init_serial(serial_port, baud)

        self.prev_angle = None
        self.prev_time = None

    def init_serial(self, port, baud):
        """
        start serial connection to arduino
        """
        # configure serial port
        self.ser = serial.Serial(
                    port=port,      # Replace with your actual serial port (COM3 on Windows, /dev/ttyUSB0 on Linux)
                    baudrate=baud,            # Baudrate (must match your device)
                    bytesize=serial.EIGHTBITS, 
                    parity=serial.PARITY_NONE, 
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1                 # Timeout in seconds for read operations
                )

        # actually initiate serial port.
        self.ser.open()
        
    def send_command(self, mode:str, servo_cmd:int, vibe_cmd:list):
        """
        used to send command to the arduino
        START_BYTE MODE SERVO VIBE1 VIBE2 ... END_BYTE
        """
        # check and translate mode
        if mode == "servo":
            mode_byte = 0
        elif mode == "vibe":
            mode_byte = 1
        elif mode == "both":
            mode_byte = 2
        elif mode == "kill":
            mode_byte = 3
        else:
            raise ValueError("Please select correct mode.")

        # check vibe commands
        for vibe in vibe_cmd:
            if vibe > MAX_VIBE:
                raise ValueError("Cannot send vibe value larger than " + str(MAX_VIBE) + ".")
            if vibe < 0:
                raise ValueError("Cannot send negative vibe.")

        # check servo commands
        if servo_cmd < SERVO_MIN or servo_cmd > SERVO_MAX:
            raise ValueError("Cannot send out of bounds servo command of " + str(servo_cmd) + ".")
        
        # construct command as a byte array, then send.
        command = bytes([254, mode_byte, servo_cmd] + vibe_cmd + [255])
        self.ser.write(command)

    def attracting_point(self, current_angle:float, goal_angle:float, increasing:bool=True):
        """
        Function used to send a command where stimulation is proportional to error.

        current_angle is the current joint angle.
        goal_angle is the target for the current time instant.
        increasing flag if true increases stimulation as angle approaches goal, with maximal stim at the goal
                        if false decreases stimulation as angle approaches goal, with minimal stim at the goal

        current_angle is stored to calculate the rate of progress, if desired. 
        this function expects that input data is filtered!
        """
        # get current error and timestamp
        err = goal_angle - current_angle
        t = time.time()

        # get the rate of change from the previous timestep
        if self.prev_angle is None:
            derr = 0
        else:
            derr = (current_angle - self.prev_angle) / (t - self.prev_time)
        



def list_ports():
    """
    Convenience function to check available ports.
    """
    ports = serial.tools.list_ports.comports()
    print("Available ports:\n------------")
    for port in ports:
        print(port)
    print("------------")