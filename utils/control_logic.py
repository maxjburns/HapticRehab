
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
    def __init__(self, serial_port, baud, control_freq=30, total_vibe_motors=8):
        
        #self.init_serial(serial_port, baud)

        self.prev_angle = None
        self.prev_time = None

        self.command_queue = []

        self.control_freq = control_freq
        self.total_vibe_motors = total_vibe_motors

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
        if len(self.command_queue) == 0:
            return False
        current_command = self.command_queue.pop(0)

        self.send_command(current_command[0], current_command[1], current_command[2:])
        return True
    
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
        
    def saltation_effect(self, motor_indices:list[int], activation_intensity, activation_time, delay):
        """
        Used to generate a series of commands to create the desired saltation illusion on the device.
        Inputs:
        motor_indices (list[str]): an ordered list of the motor indices which will be used in the saltation effect:
                                    0 --> front of shank, proximal to knee
                                    ...
                                    3 --> front of shank, distal to knee
                                    4 --> back of shank, distal to knee
                                    ...
                                    7 --> back of shank, proximal to knee

        activation_times (either list[float] or float): time in ms to buzz. 
        delays (either list[float] or float): delays in ms between sequential motor activations. 

        Outputs:
        list of modes, list of servo commands, list of vibration motor commands.
        """
        # put together the activation intensity.
        if type(activation_intensity) == float or type(activation_intensity) == int:
            intensity_ls = [activation_intensity]*len(motor_indices)
        else:
            intensity_ls = activation_intensity
        
        if len(intensity_ls) != len(motor_indices):
            raise ValueError("Vibration intensity should either be a single value, or a list of len equal to number of motor indices.")
        
        # put together the activation timing.
        if type(activation_time) == float or type(activation_time) == int:
            activation_ls = [activation_time]*len(motor_indices)
        else:
            activation_ls = activation_time
        
        if len(activation_ls) != len(motor_indices):
            raise ValueError("Motor activation times should either be a single value, or a list of len equal to number of motor indices.")
        
        # put together the delays between activations
        if type(delay) == float or type(delay) == int:
            delay_ls = [delay]*(len(motor_indices)-1)
        else:
            delay_ls = delay
        
        if len(delay_ls) != len(motor_indices) - 1:
            raise ValueError("Delays only appear between motor activations, should be one shorter than number of motors.")
        
        vibe_ls = []
        total_timesteps = 0
        for i, activation_time in enumerate(activation_ls):
            # first add the proper intensity for the motor activating at this section.
            motor_idx = motor_indices[i]
            intensity = intensity_ls[i]
            steps = int(activation_time * self.control_freq*0.001)
            
            # each timestep should last 1/control_freq
            for t in range(steps):
                new_vibe_command = [0]*self.total_vibe_motors
                new_vibe_command[motor_idx] = intensity

                vibe_ls.append(new_vibe_command)

            total_timesteps += steps

            if i >= len(delay_ls):
                continue
            # now we add empty lists for the delays.
            post_delay = delay_ls[i]
            steps = int(post_delay * self.control_freq*0.001)

            for t in range(steps):
                new_vibe_command = [0]*self.total_vibe_motors

                vibe_ls.append(new_vibe_command)

            total_timesteps += steps
        
        mode_ls = ["vibe"]*total_timesteps
        servo_ls = [90]*total_timesteps

        return mode_ls, servo_ls, vibe_ls
    

    def hold_state(self, servo_cmd:int, vibe_command:list[int], hold_time:float):
        """
        Simple helper used to generate a set of identical commands, where servo and vibe motors remain at the given values for the given time.
        This can be used to create on/off patterns if 1/freq is supplied for hold_time.
        """
        mode = ""
        for vibe in vibe_command:
            if vibe:
                mode = "vibe"

        if servo_cmd != 90:
            if mode == "vibe":
                mode = "both"
            else:
                mode == "servo"

        total_timesteps = int(hold_time*self.control_freq)

        return [mode]*total_timesteps, [servo_cmd]*total_timesteps, [vibe_command for t in range(total_timesteps)]

def merge_commands(command_lists:list[tuple]):
    """
    Function used to layer any number of commands. Layering is an additive process. Input is a list containing tuples of commands:
    command_lists = [(modes1[], servos1[], vibes1[]), 
                     (modes2[], servos2[], vibes2[]), 
                     (modes3[], servos3[], vibes3[])]
    
    for now, it is expected that these commands are fully independent. So, if one command list has motor 4 vibrating at timestep 2, no
    other command list should command motor 4 at timestep 2. In the future we should add some sort of averaging.

    output is a single tuple of commands:
    modes[], servos[], vibes[]
    """

    num_timesteps = len(command_lists[0][0])
    num_vibes = len(command_lists[0][2][0])
    out_commands = ([""]*num_timesteps, [0]*num_timesteps, [[0]*num_vibes for t in range(num_timesteps)])

    # iter through the commands
    for i in range(len(command_lists)):

        # check lengths of mode list, then servos, then vibes
        for p in range(3):
            if len(command_lists[i][p]) != num_timesteps:
                raise ValueError("Command lists must have all elements of the same length.")
            
        for t in range(num_timesteps):
            new_mode = command_lists[i][0][t]
            new_servo = command_lists[i][1][t]
            new_vibe = command_lists[i][2][t]
            
            current_mode = out_commands[0][t]
            current_servo = out_commands[1][t]
            current_vibe = out_commands[2][t]

            # mode is already assigned, we may need to change it to "both"
            if current_mode:
                # assign it as both, only if there is a mode mismatch and we haven't already selected both.
                if current_mode != new_mode and current_mode != "both":
                    out_commands[0][t] = "both"

            # mode is not yet assigned, so assign it.
            else:
                out_commands[0][t] = command_lists[i][0][t]

            # have not assigned servo for this timestep
            if not current_servo or current_servo == 90:
                out_commands[1][t] = new_servo
            # we have assigned servo already, but there is another command to be added.
            elif new_servo and new_servo != 90:
                raise ValueError("In current implementation, command lists must be independent.")
            
            for j in range(num_vibes):
                if not current_vibe[j]:
                    out_commands[2][t][j] = new_vibe[j]
                elif new_vibe[j]:
                    raise ValueError("In current implementation, command lists must be independent.") 

    return out_commands

def list_ports():
    """
    Convenience function to check available ports.
    """
    ports = serial.tools.list_ports.comports()
    print("Available ports:\n------------")
    for port in ports:
        print(port)
    print("------------")