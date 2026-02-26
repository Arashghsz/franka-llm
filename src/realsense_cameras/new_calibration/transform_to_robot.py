import pickle
from scipy.spatial.transform import Rotation as R
import numpy as np
import pyrealsense2 as rs


pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30) #set resolution and FPS here
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#Possible resolutions and FPS: https://support.intelrealsense.com/hc/en-us/community/posts/360039243533-Resolution-Configuration-Options-on-Real-Sense-D435-for-Raspberry-Pi-4-4-GB-RAM
#The exact resolution mainly determines the FPS and aspect ratio. Within the aspect ratio the image seems to get scaled so that the contect shown is the same.
profile = pipe.start(cfg)
 
color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
color_intrinsics = color_profile.get_intrinsics()

# def load_vectors():
#     """ 
#     Load rotational and translational vectors to the memory 
#     Parameters: None   
#     Returns:
#     - rotational_vector, translational vector
#     """
#     with open('rotation_vector.pkl', 'rb') as handle:
#         rotation_vectors = pickle.load(handle)

#     with open('translational_vector.pkl', 'rb') as handle:
#         translation_vectors = pickle.load(handle)

with open('rotation_vector.pkl', 'rb') as handle:
   rotation_vectors = pickle.load(handle)

with open('translational_vector.pkl', 'rb') as handle:
   translation_vectors = pickle.load(handle)

R_cam_aruco = R.from_rotvec(rotation_vectors.squeeze()).as_matrix()
R_aruco_cam = R_cam_aruco.T
t_aruco_cam = -R_aruco_cam @ translation_vectors.squeeze()
u = 605
v = 87
z = 0.50

X_cam = np.array([
    (u - color_intrinsics.ppx) * z / color_intrinsics.fx,
    (v - color_intrinsics.ppy) * z / color_intrinsics.fy,
    z
])

print('X_cam',X_cam)

X_aruco = R_aruco_cam @ X_cam + t_aruco_cam

print("Object in ArUco frame:", X_aruco)
# 0.510 is the way we calibrated the camera based on the robot arm (camera_calibration.py) which is done only once-It's done
robot_x = -X_aruco[1] + 0.510
robot_y = -X_aruco[0] - 0.01
robot_z = X_aruco[2] + 0.02
print("Object inRobot:", robot_x, robot_y, robot_z)