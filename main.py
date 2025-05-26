
from utils.pose_tracking import prepare_openpose, process_camera_frame, get_leg_angles, vec_angle
from utils.control_logic import HapticCommander, list_ports
import time

PORT = ""
BAUD = 9600
CONTROL_FREQ = 20.0
CONTROL_DT = 1.0/CONTROL_FREQ

def main():
    commander = HapticCommander(PORT, BAUD)
    prev_time = time.time()
    
    while True:
        current_time = time.time()

        if current_time - prev_time > CONTROL_DT:
            commander.attracting_point()

            prev_time = current_time



if __name__=="__main__":
    if not PORT:
        list_ports()
    else:
        main()