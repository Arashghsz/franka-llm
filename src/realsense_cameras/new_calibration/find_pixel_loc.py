import pyrealsense2 as rs
import time
import numpy as np
import cv2

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipe.start(cfg)

color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))

start_time = time.time()
warmup_seconds=2
while time.time() - start_time < warmup_seconds:
    frames = pipe.wait_for_frames()
    # discard frames

# Capture one final frame
frames = pipe.wait_for_frames()
while True:
    frames = pipe.wait_for_frames()
    color_frame = frames.get_color_frame()
    if color_frame:
        color_image = np.asanyarray(color_frame.get_data()) 
        cv2.imshow('sds',color_image)
        # wait 1 ms and check for 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break