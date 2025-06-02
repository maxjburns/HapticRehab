
from utils.pose_tracking import OpenPoseWrapper, MaskTracker
from utils.control_logic import HapticCommander, list_ports, merge_commands
import time
import cv2

import numpy as np


LEG_SELECTED = "LEFT"
DISPLAY = True
PORT = "/dev/ttyUSB0" # /dev/ttyUSB0
BAUD = 9600
WEBCAM_IDX = 0

CONTROL_FREQ = 50.0
CONTROL_DT = 1.0/CONTROL_FREQ

SAMPLE_FREQ = 10.0
SAMPLE_DT = 1.0/SAMPLE_FREQ

TRACKING_METHOD = "CV" # "OPENPOSE"
# open pose params
if TRACKING_METHOD == "OPENPOSE":
    MODEL_FOLDER = "gits/openpose/models/"
    NET_RESOLUTION = "-1x256"

# cv tracking
if TRACKING_METHOD == "CV":
    CALIB_PATH = "data/subl_calib_3.pkl"

ANKLE_BUFFER_SIZE = 20

def main():
    params = dict()

    if TRACKING_METHOD == "OPENPOSE":
    # open pose setup. It runs faster if we don't display bc no rendering needed.
        if not DISPLAY:
            params["disable_blending"] = True
            params["render_pose"] = 0
            params["display"] = 0

        # set params globally above, not here
        params["model_folder"] = MODEL_FOLDER
        params["face"] = False
        params["hand"] = False
        params["net_resolution"] = NET_RESOLUTION
        
    # construct the objects which contain the methods we use for control and ankle angle reading
    print("building haptic commander...")
    commander = HapticCommander(PORT, BAUD)


    #modes1, servo1, vibe1 = commander.saltation_effect([0, 1, 2, 3], 120, 100, 50, CONTROL_FREQ, 8)
    #modes2, servo2, vibe2 = commander.saltation_effect([4, 5, 6, 7], 120, 100, 50, CONTROL_FREQ, 8)

    #modes, servo, vibe = merge_commands([commander.saltation_effect([0, 1, 2, 3], [120, 100, 80, 120], [100, 150, 100, 90], [20, 40, 50], CONTROL_FREQ, 8),
    #                                     commander.saltation_effect([4, 5, 6, 7], 120, 100, 50, CONTROL_FREQ, 8)])
    
    #print(modes)
    #print(servo)
    #print(vibe)
    #print(""+2)
    if TRACKING_METHOD == "OPENPOSE":
        print("building openpose")
        tracking_wrapper = OpenPoseWrapper(params, WEBCAM_IDX, ANKLE_BUFFER_SIZE, sample_freq=SAMPLE_FREQ)
    
    if TRACKING_METHOD == "CV":
        print("building openpose")
        tracking_wrapper = MaskTracker(CALIB_PATH, WEBCAM_IDX, ANKLE_BUFFER_SIZE, sample_freq=SAMPLE_FREQ)

    # used to track the control and sampling loops.
    prev_sample_time = time.time()
    prev_control_time = time.time()

    # EDIT HERE FOR PROTOTYPING:
    # note: negative gains means that when error is negative, motor is active.
    #       positive gain means that when error is positive, motor is active.
    #
    #       error is goal_angle - current_angle, so if foot is too pointed, error would be positive

    goal_angle = 90

    # gains are multiplied by the angle error in the current implementation. see above for numeric info
    vibe_gains = [5.0, 5.0, 5.0, 5.0, -5.0, -5.0, -5.0, -5.0]
                  #-4.0, -4.0, -4.0, -4.0]
    servo_gain = 2.0

    gains = [servo_gain] + vibe_gains

    # this is a super simple way to get switching behavoir for testing, not intended for long term use.
    s_lim = 20
    s_idx = 0
    flag = 0

    # construct information for text overlayed on visual display.
    if DISPLAY:
        text = ""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        color = (0, 0, 0)  # White color
        thickness = 2
        position = (50, 100)  # Coordinates of the bottom-left corner of the text string

    # control loop
    while True:
        current_time = time.time()

        # we can sample and control at different frequencies. 
        # We want to sample at a higher freq than we control if possible.
        if current_time - prev_sample_time > SAMPLE_DT:
            
            # this collects ankle angle to the internal array stored within the open pose wrapper
            # we do this so that the data can be lowpass filtered when "get" is used.
            if TRACKING_METHOD == "OPENPOSE":
                tracking_wrapper.collect_ankle_angle(LEG_SELECTED, DISPLAY)

            if TRACKING_METHOD == "CV":
                tracking_wrapper.collect_ankle_angle(DISPLAY)

            # we want to render every processed frame for debugging.
            if DISPLAY:
                # openpose wrapper stores the most recent frame each time collect_ankle_angle is called.
                image = tracking_wrapper.fresh_frame.copy()

                (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)

                # set up white background
                rect_tl = (position[0], position[1] - text_height - baseline)
                rect_br = (position[0] + text_width, position[1] + baseline)

                cv2.rectangle(image, rect_tl, rect_br, (255, 255, 255), thickness=cv2.FILLED)
                cv2.putText(image, text, position, font, font_scale, color, thickness, cv2.LINE_AA)

                cv2.imshow("HapticRehab", image)
                cv2.waitKey(1)

            # reset sample clock loop
            prev_sample_time = current_time
        
        # used for the control loop, how often do we send information?
        # need to wait for ankle data to be ready (ankle buffer is full of data and ready to be filtered)
        if current_time - prev_control_time > CONTROL_DT and tracking_wrapper.ankle_ready:
            # get the current ankle angle, this is the step which applies a lowpass filter to buffered data.
            current_angle = tracking_wrapper.get_ankle_angle()
            
            # show current angle for debugging, and update image text. In collect loop we write this.
            print("current angle:", current_angle)
            if DISPLAY:
                text = "Angle: " + str(current_angle)[0:4]

            # this is where commmands can be added to the queue. currently we apply feedback every control loop, but
            # in the future we can add many commands to the queue, and execute one every time this statement is entered.
            # then, add new commands once the queue is empty.
            mode, servo_cmd, vibe_cmds = commander.attracting_point(current_angle, goal_angle, mode="both", gains=gains)
            #commander.add_commands_to_queue(mode, servo_cmd, vibe_cmds)

            # this should always be present.
            commander.send_next_command()
            
            prev_control_time = current_time

            # this is just used to have mode switching for testing motors, should be removed.
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
