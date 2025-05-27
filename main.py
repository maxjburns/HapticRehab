
from utils.pose_tracking import OpenPoseWrapper
from utils.control_logic import HapticCommander, list_ports
import time
import cv2

import numpy as np

LEG_SELECTED = "LEFT"
DISPLAY = True
PORT = "/dev/ttyUSB0"
BAUD = 9600
WEBCAM_IDX = 0

CONTROL_FREQ = 20.0
CONTROL_DT = 1.0/CONTROL_FREQ

SAMPLE_FREQ = 10.0
SAMPLE_DT = 1.0/SAMPLE_FREQ

MODEL_FOLDER = "gits/openpose/models/"
NET_RESOLUTION = "-1x256"
ANKLE_BUFFER_SIZE = 20

def main():
    params = dict()

    if not DISPLAY:
        params["disable_blending"] = True
        params["render_pose"] = 0
        params["display"] = 0

    params["model_folder"] = MODEL_FOLDER
    params["face"] = False
    params["hand"] = False
    params["net_resolution"] = NET_RESOLUTION

    print("building haptic commander...")
    commander = HapticCommander(PORT, BAUD)
    print("building openpose")
    open_pose = OpenPoseWrapper(params, WEBCAM_IDX, ANKLE_BUFFER_SIZE)

    prev_sample_time = time.time()
    prev_control_time = time.time()

    # EDIT HERE FOR PROTOTYPING:
    # note: negative gains means that when error is negative, motor is active.
    #       positive gain means that when error is positive, motor is active.
    #
    #       error is goal_angle - current_angle, so if foot is too pointed, error would be positive

    goal_angle = 90
    vibe_gains = [5.0, 5.0, 5.0, 5.0]
                  #-4.0, -4.0, -4.0, -4.0]
    servo_gain = 2.0

    gains = [servo_gain] + vibe_gains

    s_lim = 20
    s_idx = 0
    flag = 0

    if DISPLAY:
        text = ""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        color = (0, 0, 0)  # White color
        thickness = 2
        position = (50, 100)  # Coordinates of the bottom-left corner of the text string

    while True:
        current_time = time.time()

        # we can sample and control at different frequencies. 
        # We want to sample at a higher freq than we control if possible.
        if current_time - prev_sample_time > SAMPLE_DT:

            open_pose.collect_ankle_angle(LEG_SELECTED, DISPLAY)

            if DISPLAY:
                image = open_pose.fresh_frame.copy()

                (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)

                rect_tl = (position[0], position[1] - text_height - baseline)
                rect_br = (position[0] + text_width, position[1] + baseline)

                cv2.rectangle(image, rect_tl, rect_br, (255, 255, 255), thickness=cv2.FILLED)
                cv2.putText(image, text, position, font, font_scale, color, thickness, cv2.LINE_AA)

                cv2.imshow("HapticRehab", image)
                cv2.waitKey(1)

            prev_sample_time = current_time
            
        if current_time - prev_control_time > CONTROL_DT and open_pose.ankle_ready:
            current_angle = open_pose.get_ankle_angle()
            
            print("current angle:", current_angle)
            if DISPLAY:
                text = "Angle: " + str(current_angle)[0:4]
            mode, servo_cmd, vibe_cmds = commander.attracting_point(current_angle, goal_angle, mode="both", gains=gains)
            commander.add_commands_to_queue(mode, servo_cmd, vibe_cmds)
            commander.send_next_command()
            
            prev_control_time = current_time

            s_idx += 1
            if s_idx > s_lim:
                if flag:
                    flag = 0
                else:
                    flag = 1

                s_idx = 0




if __name__=="__main__":
    if not PORT:
        list_ports()
    else:
        main()
