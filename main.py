
from utils.pose_tracking import OpenPoseWrapper, MaskTracker
from utils.control_logic import HapticCommander, list_ports, merge_commands
from utils.data_collection import DataLogger
import time
import cv2

import numpy as np


DISPLAY = True
PORT = "/dev/ttyUSB0" # /dev/ttyUSB0
BAUD = 9600
WEBCAM_IDX = 2

CONTROL_FREQ = 40.0
CONTROL_DT = 1.0/CONTROL_FREQ

SAMPLE_FREQ = 40.0
SAMPLE_DT = 1.0/SAMPLE_FREQ

ANKLE_BUFFER_SIZE = 15 # samples to store for ankle angle filter
ANKLE_FILTER_CUTOFF = 10

# data collection
COLLECT_DATA = True
if COLLECT_DATA:
    SAVE_PATH = "data/subl_test_3"
    DATA_LABELS = ["epoch_time", "ankle_angle", "goal_angle", "servo_command"]
    for i in range(8):
        DATA_LABELS.append("vibe_command_" + str(i))


TRACKING_METHOD = "CV" # "OPENPOSE"
# open pose params
if TRACKING_METHOD == "OPENPOSE":
    LEG_SELECTED = "LEFT"
    MODEL_FOLDER = "gits/openpose/models/"
    NET_RESOLUTION = "-1x256"

# cv tracking
if TRACKING_METHOD == "CV":
    CALIB_PATH = "data/subl_calib_8.pkl"

def main():
    params = dict()

    upper_angle = 115.0
    lower_angle = 105.0
    MODE = "SERVO_EXPAND"
    #MODE = "VIBE_EXPAND"
    #MODE = "VIBE_PROPORTION"
    #MODE = "VIBE_PATTERN"
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

    if TRACKING_METHOD == "OPENPOSE":
        print("building openpose")
        tracking_wrapper = OpenPoseWrapper(params, WEBCAM_IDX, ANKLE_BUFFER_SIZE, lowpass_cutoff = ANKLE_FILTER_CUTOFF, sample_freq=SAMPLE_FREQ)
    
    if TRACKING_METHOD == "CV":
        print("building openpose")
        tracking_wrapper = MaskTracker(CALIB_PATH, WEBCAM_IDX, ANKLE_BUFFER_SIZE, lowpass_cutoff = ANKLE_FILTER_CUTOFF, sample_freq=SAMPLE_FREQ)

    # construct data logger
    if COLLECT_DATA:
        data_logger = DataLogger(DATA_LABELS)

    # used to track the control and sampling loops.
    prev_sample_time = time.time()
    prev_control_time = time.time()

    # construct information for text overlayed on visual display.
    if DISPLAY:
        text = ""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        color = (0, 0, 0)  # White color
        thickness = 2
        position = (50, 100)  # Coordinates of the bottom-left corner of the text string
    
    # ------------------------------ EDIT HERE FOR PROTOTYPING: ------------------------------ #
    # note: negative gains means that when error is negative, motor is active.
    #       positive gain means that when error is positive, motor is active.
    #
    #       error is goal_angle - current_angle, so if foot is too pointed, error would be positive
    #
    #     vibration motor indices for reference:
    #  0 --> front, close to knee
    #  3 --> front, close to ankle
    #  4 --> back, close to ankle
    #  7 --> back, close to knee

    goal_angle = 110
    # gains are multiplied by the angle error in the current implementation. see above for numeric info
    vibe_gains = [3.0, 3.0, 3.0, 3.0, -8.0, -8.0, -8.0, -8.0]

    servo_gain = 2.0

    gains = [servo_gain] + vibe_gains

    flag = False
    # ------------------------------ EDIT ABOVE FOR PROTOTYPING ------------------------------ #


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

            # collects from opencv hough transform tracker
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

            #           FOLLOWING ARE WHERE COMMANDS ARE SET BASED ON CURRENT ANGLE VALUE, THE IF STATEMENTS MEAN THAT ONLY
            #           THE FIRST TRIGGERS. MOVE DESIRED CONTROL STRATEGY FIRST.

                        # proportional command based on error:
            
            if MODE == "SERVO_EXPAND":
                # USE FOR DEMO - SERVO KICK EXPANDING 
                if commander.commands_in_queue() == 0:
                    # Use this format to make a switching effect (every n seconds a flag flips, causing something else to happen until it flips back)
                    # check the switch on/off example below for explanation. We don't need a timer because when the command queue runs out it means
                    # it is time to switch behavior.

                    off_time =100
                    on_time = 150

                    if current_angle < lower_angle and flag:
                        flag = False

                        mode, servo_cmd, vibe_cmds = commander.servo_kick(90, 120, 80, 300)
                        commander.add_commands_to_queue(mode, servo_cmd, vibe_cmds)
                        lower_angle -= 2.0


                    elif current_angle > upper_angle and not flag:
                        flag = True

                        mode, servo_cmd, vibe_cmds = commander.servo_kick(90, 60, 80, 300)
                        commander.add_commands_to_queue(mode, servo_cmd, vibe_cmds)
                        upper_angle += 2.0

                    else:
                        
                        mode, servo_cmd, vibe_cmds = commander.hold_state(90, [0, 0, 0, 0, 0, 0, 0, 0], 50)
                        commander.add_commands_to_queue(mode, servo_cmd, vibe_cmds)
                
            if MODE == "VIBE_EXPAND":

                # USE FOR DEMO - BOP BOP AT LIMITS THAT EXPAND
                if commander.commands_in_queue() == 0:
                    # Use this format to make a switching effect (every n seconds a flag flips, causing something else to happen until it flips back)
                    # check the switch on/off example below for explanation. We don't need a timer because when the command queue runs out it means
                    # it is time to switch behavior.


                    off_time =100
                    on_time = 150

                    if current_angle < lower_angle and flag:
                        flag = False

                        mode, servo_cmd, vibe_cmds = commander.hold_state(90, [120, 120, 120, 120, 250, 250, 250, 250], on_time)
                        commander.add_commands_to_queue(mode, servo_cmd, vibe_cmds)
                        mode, servo_cmd, vibe_cmds = commander.hold_state(90, [0, 0, 0, 0, 0, 0, 0, 0], off_time)
                        commander.add_commands_to_queue(mode, servo_cmd, vibe_cmds)
                        mode, servo_cmd, vibe_cmds = commander.hold_state(90, [120, 120, 120, 120, 250, 250, 250, 250], on_time)
                        commander.add_commands_to_queue(mode, servo_cmd, vibe_cmds)
                        mode, servo_cmd, vibe_cmds = commander.hold_state(90, [0, 0, 0, 0, 0, 0, 0, 0], off_time)
                        commander.add_commands_to_queue(mode, servo_cmd, vibe_cmds)
                        lower_angle -= 2.0


                    elif current_angle > upper_angle and not flag:
                        flag = True
                        mode, servo_cmd, vibe_cmds = commander.hold_state(90, [120, 120, 120, 120, 250, 250, 250, 250], on_time)
                        commander.add_commands_to_queue(mode, servo_cmd, vibe_cmds)
                        mode, servo_cmd, vibe_cmds = commander.hold_state(90, [0, 0, 0, 0, 0, 0, 0, 0], off_time)
                        commander.add_commands_to_queue(mode, servo_cmd, vibe_cmds)
                        mode, servo_cmd, vibe_cmds = commander.hold_state(90, [120, 120, 120, 120, 250, 250, 250, 250], on_time)
                        commander.add_commands_to_queue(mode, servo_cmd, vibe_cmds)
                        mode, servo_cmd, vibe_cmds = commander.hold_state(90, [0, 0, 0, 0, 0, 0, 0, 0], off_time)
                        commander.add_commands_to_queue(mode, servo_cmd, vibe_cmds)
                        upper_angle += 2.0

                    else:
                        
                        mode, servo_cmd, vibe_cmds = commander.hold_state(90, [0, 0, 0, 0, 0, 0, 0, 0], 50)
                        commander.add_commands_to_queue(mode, servo_cmd, vibe_cmds)

                        
            if MODE == "VIBE_PATTERN":
                # USE FOR DEMO - FREQ WHEN FAR
                if commander.commands_in_queue() == 0:
                    # Use this format to make a switching effect (every n seconds a flag flips, causing something else to happen until it flips back)
                    # check the switch on/off example below for explanation. We don't need a timer because when the command queue runs out it means
                    # it is time to switch behavior.
                    err = np.abs(current_angle - goal_angle)
                    # could set period based on error magnitude! Then you get different switching based on distance from goal
                    period = 1000 - 100*err # in ms
                    if period < 300:
                        period = 300
                    if period > 1000:
                        period = 1000
                    if err < 5:
                        flag = False
                    
                    on_servo = 90
                    on_vibes = [120.0, 120.0, 120.0, 120.0, 250.0, 250.0, 250.0, 250.0]
                    off_servo = 90
                    off_vibes = [0.0]*8
                    
                    if flag:
                        mode, servo_cmd, vibe_cmds = commander.hold_state(on_servo, on_vibes, period/2.0)
                        flag = False
                    else:
                        mode, servo_cmd, vibe_cmds = commander.hold_state(off_servo, off_vibes, period/2.0)
                        flag = True

                                                                
                    commander.add_commands_to_queue(mode, servo_cmd, vibe_cmds)


            if MODE == "VIBE_PROPORTION":
                if commander.commands_in_queue() == 0:
                    peak_vibe = 250
                    deadzone = 2.5
                    min_vibe = [50]*4 + [80]*4
                    mode, servo_cmd, vibe_cmds = commander.attracting_point(current_angle, goal_angle, mode="both", gains=gains,
                                                                            peak_vibe = peak_vibe, deadzone=deadzone, min_vibe = min_vibe)
                    commander.add_commands_to_queue(mode, servo_cmd, vibe_cmds) 
            
            # this should always be present.
            new_data_ls = [time.time(), current_angle, goal_angle] + commander.command_queue[0][1:]
            data_logger.log_data(new_data_ls)

            commander.send_next_command()
            print("ACTIVE MODE:", MODE)
            
            prev_control_time = current_time
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("You pressed 'q'. Exiting loop.")
            break

        if cv2.waitKey(1) & 0xFF == ord('s'):

            upper_angle = 115.0
            lower_angle = 105.0
            MODE = "SERVO_EXPAND"

        if cv2.waitKey(1) & 0xFF == ord('v'):

            upper_angle = 115.0
            lower_angle = 105.0
            MODE = "VIBE_EXPAND"


        if cv2.waitKey(1) & 0xFF == ord('f'):
            goal_angle = 95.0
            MODE = "VIBE_PATTERN"

        if cv2.waitKey(1) & 0xFF == ord('p'):
            goal_angle = 110.0
            MODE = "VIBE_PROPORTION"

    data_logger.save_data(SAVE_PATH + ".pkl")
    data_logger.export_as_csv(SAVE_PATH + ".csv")
    cv2.destroyAllWindows()


if __name__=="__main__":
    if not PORT:
        list_ports()
    else:
        main()
