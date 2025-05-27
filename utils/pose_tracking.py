
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

from scipy.signal import butter, lfilter

class OpenPoseWrapper:
    def __init__(self, op_params, webcam_idx, buffer_size, lowpass_cutoff:float=3.0, sample_freq:float=10.0):
        self.opWrapper = self.prepare_openpose(op_params)
        self.webcam = cv2.VideoCapture(webcam_idx)

        self.buffer_size = buffer_size
        self.ankle_angle_arr = np.zeros((self.buffer_size))
        self.ankle_arr_idx = 0

        self.b, self.a = butter_lowpass(lowpass_cutoff, sample_freq, 2)

        self.ankle_ready = False
        self.fresh_frame = None
        
    def prepare_openpose(self, params):
        """
        Constructs an openpose instance which we later call.
        """
        opWrapper = op.WrapperPython()
        opWrapper.configure(params)
        opWrapper.start()

        return opWrapper
    
    def collect_ankle_angle(self, leg_selected:str, display=False):
        """
        records ankle angle to a buffer for later filtering.
        """
        if display:
            keypoints, self.fresh_frame = self.process_camera_frame(display)
            
        else:
            keypoints = self.process_camera_frame(display)
        
        if type(keypoints) == np.ndarray and keypoints.size > 1:

            ankle_angle, _ = self.get_leg_angles(keypoints, leg_selected)
        else:
            #print("no keypoints")
            ankle_angle = None
        
        if ankle_angle == None:
            #print("failed to find ankle angle")
            return

        self.ankle_angle_arr[self.ankle_arr_idx] = ankle_angle

        self.ankle_arr_idx += 1

        if self.ankle_arr_idx >= self.buffer_size:
            self.ankle_arr_idx = 0
            self.ankle_ready = True

    def get_ankle_angle(self):
        """
        used to get filtered ankle angle
        """
        shifted_ankle_angle_arr = np.roll(self.ankle_angle_arr, -self.ankle_arr_idx-1)
        filtered_sample = lfilter(self.b, self.a, shifted_ankle_angle_arr)
        return filtered_sample[-1]

    def process_camera_frame(self, display=False):
        """
        Grabs the view from the webcam, and runs openpose to estimate keypoints.
        """
        ret, currentFrame = self.webcam.read()

        currentFrame = cv2.cvtColor(currentFrame, cv2.COLOR_BGR2RGB)
        
        datum = op.Datum()
        datum.cvInputData = currentFrame
        self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))

        keypoints = np.array(datum.poseKeypoints)

        if display:
            labeled_image = datum.cvOutputData

            labeled_image_BGR = cv2.cvtColor(labeled_image, cv2.COLOR_RGB2BGR)
            #cv2.imshow("labeled_image", labeled_image_BGR)
            
        if display:
            return keypoints, labeled_image_BGR
        
        return keypoints

    def get_leg_angles(self, keypoints, leg_selected:str):
        """
        Given a set of keypoints and a desired leg, gets the estimated joint angles in degrees.
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
            ankle_ang = np.rad2deg(vec_angle(shank_vec, foot_vec))
        else:
            ankle_ang = None

        if get_knee_angle:
            knee_ang = np.rad2deg(vec_angle(thigh_vec, shank_vec))
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

def butter_lowpass(cutoff, fs, order=2):
    nyq = 0.5 * fs  # Nyquist frequency
    normal_cutoff = cutoff / nyq
    return butter(order, normal_cutoff, btype='low', analog=False)

