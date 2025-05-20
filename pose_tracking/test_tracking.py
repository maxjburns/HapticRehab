
import sys
import os

# need to show how to find openpose python wrapper.
sys.path.append('gits/openpose/build/python')

import cv2
from sys import platform
import argparse
from openpose import pyopenpose as op
import numpy as np
import time

DISPLAY = True
DEBUG = True
WEBCAM_IDX = 2 # 2

def prepare_openpose(params):
    """
    Constructs an openpose instance which we later call.
    """
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()

    return opWrapper

def process_camera_frame(opWrapper, webcam, display=False):
    """
    Grabs the view from the intended webcam, and runs openpose to estimate keypoints.
    """
    ret, currentFrame = webcam.read()
    #if capture.isOpened():         # Checks the stream
    #    frameSize = (int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
    #                        int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
    #SCREEN_HEIGHT = frameSize[0]
    #SCREEN_WIDTH = frameSize[1]

    currentFrame = cv2.cvtColor(currentFrame, cv2.COLOR_BGR2RGB)
    
    datum = op.Datum()
    datum.cvInputData = currentFrame
    opWrapper.emplaceAndPop(op.VectorDatum([datum]))

    keypoints = np.array(datum.poseKeypoints)
    if display:
        labeled_image = datum.cvOutputData

        labeled_image_BGR = cv2.cvtColor(labeled_image, cv2.COLOR_RGB2BGR)
        cv2.imshow("labeled_image", labeled_image_BGR)
        cv2.waitKey(1)
    return keypoints

def get_leg_angles(keypoints, leg_selected="LEFT"):
    """
    Given a set of keypoints and a desired leg, gets the estimated joint angles.
    """

    # Focus on desired leg
    if leg_selected == "LEFT":
        hip_idx = 12
        knee_idx = 13
        ankle_idx = 14
        big_toe_idx = 19
        small_toe_idx = 20
        heel_idx = 21
    
    elif leg_selected == "RIGHT":
        hip_idx = 9
        knee_idx = 10
        ankle_idx = 11
        big_toe_idx = 22
        small_toe_idx = 23
        heel_idx = 24

    else:
        raise ValueError("Please select a leg.")
    
    # get joint positions from keypoints
    hip_pos = keypoints[0][hip_idx]
    knee_pos = keypoints[0][knee_idx]
    ankle_pos = keypoints[0][ankle_idx]
    big_toe_pos = keypoints[0][big_toe_idx]
    small_toe_pos = keypoints[0][small_toe_idx]
    heel_pos = keypoints[0][heel_idx]

    # for toe position, pick whichever the model is more confident about
    if big_toe_pos[2] > small_toe_pos[2]:
        toe_pos = big_toe_pos
    else:
        toe_pos = small_toe_pos

    # check if any joint is not visible. If any are not, then turn off the relevant angles.
    get_ankle_angle = True
    get_knee_angle = True

    if np.linalg.norm(hip_pos[0:2]) < 0.01:
        print("ERROR: " + leg_selected + " HIP NOT VISIBLE")
        get_knee_angle = False

    if np.linalg.norm(knee_pos[0:2]) < 0.01:
        print("ERROR: " + leg_selected + " KNEE NOT VISIBLE")
        get_knee_angle = False
        get_ankle_angle = False

    if np.linalg.norm(ankle_pos[0:2]) < 0.01:
        print("ERROR: " + leg_selected + " ANKLE NOT VISIBLE")
        get_ankle_angle = False

    if np.linalg.norm(toe_pos[0:2]) < 0.01:
        print("ERROR: " + leg_selected + " TOE NOT VISIBLE")
        get_ankle_angle = False

    # defined vectors pointing "down the chain" from previous joint, starting at torso
    thigh_vec = knee_pos - hip_pos
    shank_vec = ankle_pos - knee_pos
    foot_vec = toe_pos - heel_pos 

    # if anything was not in frame and couldn't be estimated, do not return the angle.
    if get_ankle_angle:
        ankle_ang = vec_angle(shank_vec, foot_vec)
    else:
        ankle_ang = None

    if get_knee_angle:
        knee_ang = vec_angle(thigh_vec, shank_vec)
    else:
        knee_ang = None

    return ankle_ang, knee_ang

def vec_angle(u, v):
    """
    Takes two vectors, returns the minimum angle between them.
    """
    cos_theta = np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))
    cos_theta = np.clip(cos_theta, -1.0, 1.0) 
    return np.arccos(cos_theta)


if __name__ == "__main__":
    params = dict()

    if not DISPLAY:
        params["disable_blending"] = True
        params["render_pose"] = 0
        params["display"] = 0

    params["model_folder"] = "gits/openpose/models/"
    params["face"] = False
    params["hand"] = False
    params["net_resolution"] = "-1x256"

    opWrapper = prepare_openpose(params)
    webcam = cv2.VideoCapture(WEBCAM_IDX)
    
    i = 0
    i_save = 20*10
    out = np.zeros((i_save, 3))
    start_time = time.time()
    while True:
        keypoints = process_camera_frame(opWrapper, webcam, DISPLAY)
        if type(keypoints) == np.ndarray and keypoints.size > 1:
            if DEBUG: print("Person detected.")
            ankle_angle, knee_angle = get_leg_angles(keypoints)
            if ankle_angle != None:
                ankle_angle = np.rad2deg(ankle_angle)
            if knee_angle != None:
                knee_angle = np.rad2deg(knee_angle)

            if DEBUG: print("ANGLES:", "ankle", ankle_angle, "knee", knee_angle)

            out[i, 0] = time.time() - start_time
            out[i, 1] = ankle_angle
            out[i, 2] = knee_angle
            i += 1
        
        else:
            if DEBUG: print("No person detected.")

            
        if i >= i_save:
            i = 0
            print("SAVING DATA...")
            np.savetxt("output.csv", out, delimiter=",")

