 
import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import pickle
#based on https://www.geeksforgeeks.org/detecting-aruco-markers-with-opencv-and-python-1/#case-study-pose-estimation-using-aruco-markers and https://www.geeksforgeeks.org/camera-position-in-world-coordinate-from-cv-solvepnp/
 
 
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30) #set resolution and FPS here
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#Possible resolutions and FPS: https://support.intelrealsense.com/hc/en-us/community/posts/360039243533-Resolution-Configuration-Options-on-Real-Sense-D435-for-Raspberry-Pi-4-4-GB-RAM
#The exact resolution mainly determines the FPS and aspect ratio. Within the aspect ratio the image seems to get scaled so that the contect shown is the same.
profile = pipe.start(cfg)
 
color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
color_intrinsics = color_profile.get_intrinsics()
#see https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.intrinsics.html#pyrealsense2.intrinsics
print("color camera intrinsics: ", color_intrinsics)
print("fx: ", color_intrinsics.fx)
print("fy: ", color_intrinsics.fy)
print("ppx: ", color_intrinsics.ppx) #VERIFY: corresponds to cx in https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html ?
print("ppy: ", color_intrinsics.ppy) #VERIFY: corresponds to cy in https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html ?
print("distortion coeffs: ", color_intrinsics.coeffs)
 
fx = color_intrinsics.fx
fy = color_intrinsics.fy
cx = color_intrinsics.ppx
cy = color_intrinsics.ppy
 
dist_coeffs = np.array(color_intrinsics.coeffs)
camera_matrix = np.array([
    [fx, 0, cx],
    [0, fy, cy],
    [0, 0,  1]
])
 
# Define the dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
 
# Create the ArUco detector
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Warmup - discard first few frames
print("Warming up camera...")
import time
for _ in range(30):
    try:
        pipe.wait_for_frames()
    except:
        time.sleep(0.1)
print("Camera ready!")
 
while(True):
    frame = pipe.wait_for_frames()
    color_frame = frame.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    
    depth_frame = frame.get_depth_frame()
    
    # Detect the markers
    corners, ids, rejected = detector.detectMarkers(color_image)
    #print("corners---", corners)
    if ids is not None:
        #print("ids: ", ids[0])
        cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
 
 
        marker_len = 0.094 # meters
        #marker_len = 9.4 # centimeters
        #marker_len = 94 # millimeters
 
        #NOTE: see how the object coordinate system is set in the c++ tutorial https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html > origin placed at middle of marker
        object_points = np.array([[
             [-marker_len/2, -marker_len/2, 0], [marker_len/2, -marker_len/2, 0], [marker_len/2, marker_len/2, 0], [-marker_len/2, marker_len/2, 0],
        ]], dtype=float)
 
 
        #see https://www.geeksforgeeks.org/camera-position-in-world-coordinate-from-cv-solvepnp/
        success, rotation_vectors, translation_vectors = cv2.solvePnP(
            object_points, corners[0], camera_matrix, dist_coeffs
        )
        cv2.drawFrameAxes(color_image, camera_matrix, dist_coeffs, rotation_vectors, translation_vectors, marker_len * 1.5, 2)
 
        #NOTE: Rotation vector
        #print(f"x: {translation_vectors[0][0]:.2f}, y: {translation_vectors[1][0]:.2f}, z: {translation_vectors[2][0]:.2f}, rx: {rotation_vectors[0][0]:.2f}, ry: {rotation_vectors[1][0]:.2f}, rz: {rotation_vectors[2][0]:.2f}")
 
        #NOTE: euler angles
        rotation = R.from_rotvec(np.squeeze(rotation_vectors))
        euler = rotation.as_euler('xyz', degrees=True)
        print(euler)
        #NOTE: line below prints transform from camera frame (z out from camera, y downawrds, x to the right) to the fram eof the ArUco tag
        #print(f"x: {translation_vectors[0][0]:.2f}, y: {translation_vectors[1][0]:.2f}, z: {translation_vectors[2][0]:.2f}, rx: {euler[0]:.2f}, ry: {euler[1]:.2f}, rz: {euler[2]:.2f}")
 
        #Below is transfrmation from AruCo frame to Camera (see https://hades.mech.northwestern.edu/images/7/7f/MR.pdf for inverse for homogenous transformation matrix at Equation 3.64 and Proposition 3.3)
        inv_rot = rotation.inv()
        t_old = np.squeeze(translation_vectors)
        t_new = -inv_rot.as_matrix() @ np.squeeze(translation_vectors) 
        euler_new = inv_rot.as_euler('xyz', degrees=True)
        #print(f"x: {t_new[0]:.2f}, y: {t_new[1]:.2f}, z: {t_new[2]:.2f}, rx: {euler_new[0]:.2f}, ry: {euler_new[1]:.2f}, rz: {euler_new[2]:.2f}")
        # Apply translation offsets to t_new
        translated_t_new = np.copy(t_new) # Create a copy to avoid modifying the original t_new
        translated_t_new[0] -= 0.1 # Translate along x-axis by -1.32m
        translated_t_new[2] -= 0.0 # Translate along z-axis by -0.64m
        # save the translation and rotation vector. the translation will be offset based on the distance between the marker and the robot frame
        with open('rotation_vector.pkl', 'wb') as handle:
            pickle.dump(rotation_vectors, handle, protocol=pickle.HIGHEST_PROTOCOL)
        with open('translational_vector.pkl', 'wb') as handle:
            pickle.dump(translation_vectors, handle, protocol=pickle.HIGHEST_PROTOCOL)
        print(f"Camera in ArUco frame (translated): x: {translated_t_new[0]:.2f}, y: {translated_t_new[1]:.2f}, z: {translated_t_new[2]:.2f}, rx: {euler_new[0]:.2f}, ry: {euler_new[1]:.2f}, rz: {euler_new[2]:.2f}")


    height, width, channels = color_image.shape
    #color_image = cv2.line(color_image, (int(width/2),0), (int(width/2),int(height)), color=(0, 0, 255)) #vertical center-line
    color_image = cv2.line(color_image, (0,int(height/2)), (int(width),int(height/2)), color=(0, 0, 255)) #horizontal centerl line
    
    #crop to same size used with the VLA
    x_center = int(np.size(color_image,1)/2)
    color_image = color_image[:, x_center-240:x_center+240]
 
    cv2.imshow('Detected Markers', color_image)
 
    if cv2.waitKey(1) == ord('q'):
        break
