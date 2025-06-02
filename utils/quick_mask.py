# used to take an image and quickly set filtering parameters to be used in the main opencv pose tracking.

import cv2
import numpy as np
import pickle
from utils.pose_tracking import sort_leg_points

def save_webcam_image(save_path, webcam_idx):
    # Open a connection to the specified webcam 
    cap = cv2.VideoCapture(webcam_idx)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        exit()

    # Capture frame
    ret, frame = cap.read()
    if not ret:
        raise ValueError("Failed to grab frame.")

    # Save as an image file
    cv2.imwrite(save_path, frame)
    print("Image saved as " + save_path)

    # Release the capture
    cap.release()


def launch_mask_picker(load_path=None, webcam_idx=None, starter_params=None, param_save_path=None):
    def nothing(x):
        pass

    if starter_params:
        with open(starter_params, 'rb') as f:
            in_dict = pickle.load(f)

        mask = in_dict["mask"]
        kernel_size = in_dict["kernel_size"]
        param1 = in_dict["param1"]
        param2 = in_dict["param2"]
        minDistance = in_dict["minDistance"]
        minRadius = in_dict["minRadius"]
        maxRadius = in_dict["maxRadius"]

    # Load image
    image = cv2.imread(load_path)
    cap = None
    if image is None or not image:
        print("no image supplied, starting live feed...")
        cap = cv2.VideoCapture(webcam_idx)

    if mask==None:
        mask = [[0, 0, 0], [179, 179, 255], [0, 0, 0], [179,  179, 255]]

    # Create window
    cv2.namedWindow('Mask Controls')
    cv2.namedWindow('Opening Menu')
    cv2.namedWindow('Hough Menu')

    # menu for opening kernel size
    cv2.createTrackbar('Param 1', 'Hough Menu', param1, 50, nothing)
    cv2.createTrackbar('Param 2', 'Hough Menu', param2, 50, nothing)
    cv2.createTrackbar('Min Distance', 'Hough Menu', minDistance, 500, nothing)
    cv2.createTrackbar('Min Radius', 'Hough Menu', minRadius, 30, nothing)
    cv2.createTrackbar('Max Radius', 'Hough Menu', maxRadius, 30, nothing)

    # menu for opening kernel size
    cv2.createTrackbar('Kernel Size', 'Opening Menu', kernel_size, 10, nothing)

    # Create trackbars for Range 1 (lower reds)
    cv2.createTrackbar('H1 Min', 'Mask Controls', mask[0][0], 179, nothing)
    cv2.createTrackbar('H1 Max', 'Mask Controls', mask[1][0], 179, nothing)
    cv2.createTrackbar('S1 Min', 'Mask Controls', mask[0][1], 255, nothing)
    cv2.createTrackbar('S1 Max', 'Mask Controls', mask[1][1], 255, nothing)
    cv2.createTrackbar('V1 Min', 'Mask Controls', mask[0][2], 255, nothing)
    cv2.createTrackbar('V1 Max', 'Mask Controls', mask[1][2], 255, nothing)

    # Create trackbars for Range 2 (upper reds)
    cv2.createTrackbar('H2 Min', 'Mask Controls', mask[2][0], 179, nothing)
    cv2.createTrackbar('H2 Max', 'Mask Controls', mask[3][0], 179, nothing)
    cv2.createTrackbar('S2 Min', 'Mask Controls', mask[2][1], 255, nothing)
    cv2.createTrackbar('S2 Max', 'Mask Controls', mask[3][1], 255, nothing)
    cv2.createTrackbar('V2 Min', 'Mask Controls', mask[2][2], 255, nothing)
    cv2.createTrackbar('V2 Max', 'Mask Controls', mask[3][2], 255, nothing)

    while True:
        if cap is None:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        else:
            ret, image = cap.read()
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)



        param1 = cv2.getTrackbarPos('Param 1', 'Hough Menu')
        param2 = cv2.getTrackbarPos('Param 2', 'Hough Menu')
        minDistance = cv2.getTrackbarPos('Min Distance', 'Hough Menu')
        minRadius = cv2.getTrackbarPos('Min Radius', 'Hough Menu')
        maxRadius = cv2.getTrackbarPos('Max Radius', 'Hough Menu')


        kernel_size = cv2.getTrackbarPos('Kernel Size', 'Opening Menu')

        # Get positions for Range 1
        h1_min = cv2.getTrackbarPos('H1 Min', 'Mask Controls')
        h1_max = cv2.getTrackbarPos('H1 Max', 'Mask Controls')
        s1_min = cv2.getTrackbarPos('S1 Min', 'Mask Controls')
        s1_max = cv2.getTrackbarPos('S1 Max', 'Mask Controls')
        v1_min = cv2.getTrackbarPos('V1 Min', 'Mask Controls')
        v1_max = cv2.getTrackbarPos('V1 Max', 'Mask Controls')

        # Get positions for Range 2
        h2_min = cv2.getTrackbarPos('H2 Min', 'Mask Controls')
        h2_max = cv2.getTrackbarPos('H2 Max', 'Mask Controls')
        s2_min = cv2.getTrackbarPos('S2 Min', 'Mask Controls')
        s2_max = cv2.getTrackbarPos('S2 Max', 'Mask Controls')
        v2_min = cv2.getTrackbarPos('V2 Min', 'Mask Controls')
        v2_max = cv2.getTrackbarPos('V2 Max', 'Mask Controls')

        # Define ranges
        lower_red1 = np.array([h1_min, s1_min, v1_min])
        upper_red1 = np.array([h1_max, s1_max, v1_max])

        lower_red2 = np.array([h2_min, s2_min, v2_min])
        upper_red2 = np.array([h2_max, s2_max, v2_max])

        # Create two masks and combine
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        combined_mask = cv2.bitwise_or(mask1, mask2)

        # Apply combined mask
        result = cv2.bitwise_and(image, image, mask=combined_mask)
        
        # Erosion then dilation AKA opening operation
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        opened_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)

        # Apply combined mask
        opened_result = cv2.bitwise_and(image, image, mask=opened_mask)
        gray_result = cv2.cvtColor(opened_result, cv2.COLOR_BGR2GRAY)

        gray_result = cv2.medianBlur(gray_result,5)

        circles = cv2.HoughCircles(gray_result,cv2.HOUGH_GRADIENT,1,minDistance,
                            param1=param1,param2=param2,minRadius=minRadius,maxRadius=maxRadius)
        
        #print(circles)
        if circles is not None:

            print(circles)
            circles = np.uint16(np.around(circles))
            cimg = opened_mask.copy()
            for i in circles[0,:]:
                # draw the outer circle
                cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
            
            cv2.imshow('detected circles',cimg)

        # Show images
        cv2.imshow('Original Image', image)
        #cv2.imshow('Combined Mask', combined_mask)
        cv2.imshow('Masked Result', result)
        cv2.imshow('After Opening', opened_result)

        # Break loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    print("MASK 1:")
    print(lower_red1, upper_red1)
    print("MASK 2:")
    print(lower_red2, upper_red2)
    print("KERNEL SIZE:")
    print(kernel_size)
    print("HOUGH PARAMS:")
    print(param1, param2, minDistance, minRadius, maxRadius)

    out_dict = {"mask": [lower_red1, upper_red1, lower_red2, upper_red2],
                "kernel_size": kernel_size,
                "param1": param1,
                "param2": param2,
                "minDistance": minDistance,
                "minRadius": minRadius,
                "maxRadius": maxRadius}
    
    if param_save_path:
        with open(param_save_path, 'wb') as f:
            pickle.dump(out_dict, f)
    # Clean up
    cv2.destroyAllWindows()

    return [lower_red1, upper_red1, lower_red2, upper_red2], kernel_size, [param1, param2, minDistance, minRadius, maxRadius]
    
def test_mask(webcam_idx, mask):
    # Define ranges
    lower_mask1 = np.array(mask[0])
    upper_mask1 = np.array(mask[1])

    if len(mask) > 2:
        lower_mask2 = np.array(mask[2])
        upper_mask2 = np.array(mask[3])
        
    cap = cv2.VideoCapture(webcam_idx)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        exit()

    while True:

        # Capture frame
        ret, currentFrame = cap.read()
        if not ret:
            raise ValueError("Failed to grab frame.")
        
        currentFrameHSV = cv2.cvtColor(currentFrame, cv2.COLOR_BGR2HSV)

        # Create two masks and combine
        mask1 = cv2.inRange(currentFrameHSV, lower_mask1, upper_mask1)
        mask2 = cv2.inRange(currentFrameHSV, lower_mask2, upper_mask2)
        combined_mask = cv2.bitwise_or(mask1, mask2)

        # Apply combined mask
        masked_im = cv2.bitwise_and(currentFrame, currentFrame, mask=combined_mask)

        cv2.imshow("masked feed", masked_im)
        cv2.waitKey(1)

if __name__=="__main__":
    im_path = "data/webcam_test_1.jpg"
    webcam_idx = 0
    mask = [[  0,  72, 164], [  2, 211, 255], [171, 87, 210], [179, 206, 255]]
    old_calib = "data/subl_calib_3.pkl"
    #test_mask(webcam_idx, mask)
    #save_webcam_image(im_path, webcam_idx)
    launch_mask_picker(None, webcam_idx, old_calib, "data/subl_calib_4.pkl")