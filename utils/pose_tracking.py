
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
import pickle

class MaskTracker:
    def __init__(self, calib_path, webcam_idx, buffer_size, lowpass_cutoff:float=3.0, sample_freq:float=10.0):
    
        self.webcam = cv2.VideoCapture(webcam_idx)

        self.buffer_size = buffer_size
        self.ankle_angle_arr = np.zeros((self.buffer_size))
        self.ankle_arr_idx = 0

        self.b, self.a = butter_lowpass(lowpass_cutoff, sample_freq, 2)

        self.ankle_ready = False
        self.fresh_frame = None
        with open(calib_path, 'rb') as f:
            self.calib = pickle.load(f)

    def collect_ankle_angle(self, display=False):
        """
        records ankle angle to a buffer for later filtering.
        """
        if display:
            points, self.fresh_frame = self.process_camera_frame(display)
            
        else:
            points = self.process_camera_frame(display)
        
        if type(points) == np.ndarray and points.size > 1:

            ankle_angle = self.get_leg_angles(points)
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
        Grabs the view from the webcam, and uses masks and hough transforms to estimate marker positions.
        """
        ret, currentFrame = self.webcam.read()

        currentFrameHSV = cv2.cvtColor(currentFrame, cv2.COLOR_BGR2HSV)

        # mask params
        lower_mask1 = self.calib["mask"][0]
        upper_mask1 = self.calib["mask"][1]

        lower_mask2 = self.calib["mask"][2]
        upper_mask2 = self.calib["mask"][3]

        # opening params
        kernel_size = self.calib["kernel_size"]

        # hough params
        param1 = self.calib["param1"]
        param2 = self.calib["param2"]
        minDistance = self.calib["minDistance"]
        minRadius = self.calib["minRadius"]
        maxRadius = self.calib["maxRadius"]

        # Create two masks and combine
        mask1 = cv2.inRange(currentFrameHSV, lower_mask1, upper_mask1)
        mask2 = cv2.inRange(currentFrameHSV, lower_mask2, upper_mask2)
        combined_mask = cv2.bitwise_or(mask1, mask2)
        
        # Erosion then dilation AKA opening operation
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        opened_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)

        # Apply combined mask
        opened_result = cv2.bitwise_and(currentFrame, currentFrame, mask=opened_mask)
        gray_result = cv2.cvtColor(opened_result, cv2.COLOR_BGR2GRAY)

        gray_result = cv2.medianBlur(gray_result,5)

        circles = cv2.HoughCircles(gray_result,cv2.HOUGH_GRADIENT,1,minDistance,
                            param1=param1,param2=param2,minRadius=minRadius,maxRadius=maxRadius)
        
        

        points = np.zeros((4, 2))
        if circles is not None:

            circles = np.uint16(np.around(circles))
            cimg = opened_mask.copy()
            for i in circles[0,:]:
                # draw the outer circle
                cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
            
            cv2.imshow('detected circles',cimg)

            for i, circle in enumerate(circles[0,:]):
                # draw the outer circle
                if i >= 4:
                    print("MASKING ERROR, TOO MANY POINTS DETECTED!")
                    if display:
                        return None, currentFrame
                    return None
                points[i, 0] = circle[0]
                points[i, 1] = circle[1]

            sorted_points = sort_leg_points(points)

        else:
            sorted_points = points

        if display: 
            labeled_image = currentFrame.copy()
            if circles is not None and circles.shape[1] == 4:
                # draw lines on image
                print(sorted_points)
                for i in range(sorted_points.shape[0]-1):
                    cv2.line(labeled_image, sorted_points[i].astype(int), sorted_points[i+1].astype(int), color=(255, 0, 0), thickness=2)

                # draw points on image
                for point in sorted_points:
                    cv2.circle(labeled_image, point.astype(int), radius=5, color=(0, 255, 0), thickness=-1)  # filled green point

            
            return sorted_points, labeled_image
        
        return sorted_points

    def get_leg_angles(self, sorted_points):
        """
        Given a set of sorted_points, gets the estimated joint angles in degrees.
        """
        
        # get joint positions from keypoints
        upper_shank_pos = sorted_points[3]
        lower_shank_pos = sorted_points[2]
        heel_pos = sorted_points[1]
        toe_pos = sorted_points[0]

        # check if any joint is not visible. If any are not, then turn off the relevant angles.
        get_ankle_angle = True

        if np.linalg.norm(upper_shank_pos[0:2]) < 0.01:
            print("ERROR: UPPER SHANK NOT VISIBLE")
            get_knee_angle = False

        if np.linalg.norm(lower_shank_pos[0:2]) < 0.01:
            print("ERROR: LOWER SHANK NOT VISIBLE")
            get_knee_angle = False
            get_ankle_angle = False

        if np.linalg.norm(heel_pos[0:2]) < 0.01:
            print("ERROR: HEEL NOT VISIBLE")
            get_ankle_angle = False

        if np.linalg.norm(toe_pos[0:2]) < 0.01:
            print("ERROR: TOE NOT VISIBLE")
            get_ankle_angle = False

        # defined vectors pointing "down the chain" from previous joint, starting at torso
        shank_vec = upper_shank_pos - lower_shank_pos
        foot_vec = toe_pos - heel_pos 

        # if anything was not in frame and couldn't be estimated, do not return the angle.
        if get_ankle_angle:
            ankle_ang = np.rad2deg(vec_angle(shank_vec, foot_vec))
        else:
            ankle_ang = None

        return ankle_ang

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

def sort_leg_points(points):
    """
    Utility function to sort out which points belong to which segments. This may be the most horrific thing I have created.

    returns sorted points in order: toe, heel, lower shank, upper shank
    """
    # we need to assign points to the correct indices:
    dist = []
    indices = [] 
    sorted_points = np.zeros((4, 2))

    # loop through all collected points

    for i, point_1 in enumerate(points):
        for j, point_2 in enumerate(points[i+1:]):
            dist.append(np.linalg.norm(point_2-point_1))
            indices.append({i, j+i+1})

    sorted_indices = [v for s, v in sorted(zip(dist, indices))]
    upper_shank_idx = sorted_indices[-1].intersection(sorted_indices[-2])
    
    upper_shank_idx = upper_shank_idx.pop()
    
    # starting at the closest point pairs, loop through until you find the point closest to upper shank idx
    for i in range(len(sorted_indices)):
        pair = sorted_indices[i]

        if upper_shank_idx in pair:
            lower_shank_idx = pair - {upper_shank_idx}
            lower_shank_idx = lower_shank_idx.pop()
            break

    # starting at the closest point pairs, loop through until you find the point closest to lower shank idx
    for i in range(len(sorted_indices)):
        pair = sorted_indices[i]

        if lower_shank_idx in pair:
            heel_idx = pair - {lower_shank_idx}
            heel_idx = heel_idx.pop()
            break
    
    toe_idx = {0, 1, 2, 3} - {upper_shank_idx, lower_shank_idx, heel_idx}
    toe_idx = toe_idx.pop()

    sorted_points[0] = points[toe_idx]
    sorted_points[1] = points[heel_idx]
    sorted_points[2] = points[lower_shank_idx]
    sorted_points[3] = points[upper_shank_idx]

    return sorted_points

    