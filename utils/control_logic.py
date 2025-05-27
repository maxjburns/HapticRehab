
import serial
import serial.tools.list_ports
import time
import numpy as np

START_BYTE = 254
END_BYTE = 255
MAX_VIBE = 253
SERVO_MIN = 0
SERVO_MAX = 180
DEBUG = True

class HapticCommander:
    def __init__(self, serial_port, baud):
        
        self.init_serial(serial_port, baud)

        self.prev_angle = None
        self.prev_time = None

        self.command_queue = []

        self.debug = DEBUG

    def init_serial(self, port, baud):
        """
        start serial connection to arduino
        """

        print("trying to start serial connection...")
        # configure serial port
        try:
            self.ser = serial.Serial(
                        port=port,      # Replace with your actual serial port (COM3 on Windows, /dev/ttyUSB0 on Linux)
                        baudrate=baud,            # Baudrate (must match your device)
                        bytesize=serial.EIGHTBITS, 
                        parity=serial.PARITY_NONE, 
                        stopbits=serial.STOPBITS_ONE,
                        timeout=1                 # Timeout in seconds for read operations
                    )
        except:
            raise ValueError("Serial port " + str(port) + " could not be opened!")
        print("serial connection sucessful!")

    def add_commands_to_queue(self, modes:list[str]=["both"], servo_cmds:list[int]=[90], vibe_cmds:list[list[int]]=[[0]]):
        for i in range(len(modes)):
            mode = modes[i]
            servo_cmd = servo_cmds[i]
            vibe_cmd = vibe_cmds[i]

            if mode not in ["servo", "vibe", "both"]:
                raise NotImplementedError("Not acceptable mode.")
            
            self.command_queue.append([mode, servo_cmd] + vibe_cmd)

    def send_next_command(self):
        """
        Used to send the next command in the queue, should be called at the desired control frequency.
        """
        if self.debug: print("command queue size:", len(self.command_queue))

        current_command = self.command_queue.pop(0)

        self.send_command(current_command[0], current_command[1], current_command[2:])

    def send_command(self, mode:str, servo_cmd:int=90, vibe_cmd:list[int]=[0]):
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

        if self.debug: print("sending command:", mode, servo_cmd, vibe_cmd, "\n\tas byte arr:", command)
        self.ser.write(command)
        
    def attracting_point(self, current_angle:float, goal_angle:float, increasing:bool=True, mode:str="servo", gains:list[float]=[1.5],
                         peak_vibe:int=200):
        """
        Function used to generate a command where stimulation is proportional to error.

        current_angle is the current joint angle.
        goal_angle is the target for the current time instant.
        increasing flag if true increases stimulation as angle approaches goal, with maximal stim at the goal (vibe only)
                        if false decreases stimulation as angle approaches goal, with minimal stim at the goal (vibe only)
        mode is "vibe" "servo" or "both" depending on the type of stimulation desired.
        gain is what to multiply angle error by to achieve a 0-253 command (or 0-180 for the servo) it is a list
                        which should be ordered relative to the index of the servo and motors.
        
        returns mode, servo_command, vibe_commands which should be then added to the command queue.

        current_angle is also stored to calculate the rate of progress, if desired. 
        this function expects that input data is filtered!
        """
        if mode not in ["servo", "vibe", "both"]:
            raise NotImplementedError("Unacceptable mode.")
        
        # get current error and timestamp
        err = goal_angle - current_angle
        t = time.time()

        # get the rate of change from the previous timestep
        if self.prev_angle is None:
            derr = 0
        else:
            derr = (current_angle - self.prev_angle) / (t - self.prev_time)
        
        # TODO implement rate of change in error (damping?)
        servo_command = 0
        vibe_command = [0]*(len(gains)-1)
        
        if mode == "servo" or mode=="both":
            servo_command = 90 + gains[0] * err
            if servo_command > 180: servo_command = 180
            elif servo_command < 0: servo_command = 0
        
        if mode == "vibe" or mode=="both":
            vibe_idx = 0
            for gain in gains[1:]:
                if increasing: 
                    vibe_command[vibe_idx] = int(gain*err)

                else: 
                    vibe_command[vibe_idx] = int(peak_vibe - np.abs(gain*err))

                # trim vibe commands if too large or negative
                if vibe_command[vibe_idx] > peak_vibe:
                    vibe_command[vibe_idx] = peak_vibe

                elif vibe_command[vibe_idx] < 0:
                    vibe_command[vibe_idx] = 0 

                vibe_idx += 1

        return [mode], [int(servo_command)], [vibe_command]
        



def list_ports():
    """
    Convenience function to check available ports.
    """
    ports = serial.tools.list_ports.comports()
    print("Available ports:\n------------")
    for port in ports:
        print(port)
    print("------------")