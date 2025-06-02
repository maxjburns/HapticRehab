from utils.quick_mask import launch_mask_picker


if __name__=="__main__":
    webcam_idx = 2 # or 2
    old_calib = "data/subl_calib_3.pkl"
    new_calib_path = "data/calib.pkl"

    launch_mask_picker(None, webcam_idx, old_calib, new_calib_path)