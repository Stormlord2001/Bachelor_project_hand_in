import cv2 
import numpy as np
import djitellopy as tello
import keyboard
import threading
import matplotlib.pyplot as plt
import time
from datetime import datetime
import pandas as pd

from pose_estimation3 import *
from TelloClass import *
from pid import *
from pathfinder import *
from control_threads import *
from Human_pose_estimation import *

# Run the drone or the camera
run_drone = True


ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4x4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000
}

aruco_type = "DICT_4X4_50"

arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
arucoParams = cv2.aruco.DetectorParameters()
arucoParams.useAruco3Detection = False
arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

# Camera calibration
intrinsic_camera = np.array(((919.46308775, 0, 485.14496399),(0, 915.9457444, 384.1439899),(0, 0, 1)))
distortion = np.array((-0.001065, -0.0662876, 0.00030168, 0.00281735, 0.21070331))

msize = 0.136
ftcvt = -0.106
ftcvb = 0.104
ftch = 0.071
object_points = np.float64([[-msize/2-ftch, msize/2-ftcvt, 0], [msize/2-ftch, msize/2-ftcvt, 0], [msize/2-ftch, -msize/2-ftcvt, 0], [-msize/2-ftch, -msize/2-ftcvt, 0],
                             [-msize/2+ftch, msize/2-ftcvt, 0], [msize/2+ftch, msize/2-ftcvt, 0], [msize/2+ftch, -msize/2-ftcvt, 0], [-msize/2+ftch, -msize/2-ftcvt, 0],
                             [-msize/2-ftch, msize/2-ftcvb, 0], [msize/2-ftch, msize/2-ftcvb, 0], [msize/2-ftch, -msize/2-ftcvb, 0], [-msize/2-ftch, -msize/2-ftcvb, 0],
                             [-msize/2+ftch, msize/2-ftcvb, 0], [msize/2+ftch, msize/2-ftcvb, 0], [msize/2+ftch, -msize/2-ftcvb, 0], [-msize/2+ftch, -msize/2-ftcvb, 0]])

if run_drone:
    drone = Drone()
    drone.camera_on()
else:
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

pos_dict = {0:  [0.00, 0.00, 1., 0.0],
            4:  [1.00, 2.89,1.,np.pi],
            8:  [-0.46, 5.82, 1., -np.pi/2],
            12: [0.54, 2.96, 1., 0],
            16: [3.60, 6.99, 1., np.pi],
            24: [4.08, 2.35, 1., np.pi/2]}


pose_estimator = PoseEstimation3(pos_dict, arucoDict, arucoParams, intrinsic_camera, distortion, object_points, 4, 4, use_kalman=True, L = 2, sigma_z_sq = 2, sigma_a_sq = 0.01, second_order_kalman=False, adaptive_kalman=True)
path_finder = Pathfinder(pos_dict)

# Define the connections between the markers
connections = np.array([[0, 4], [4, 24], [24, 16], [16, 8], [8, 12]])
graph = Graph(connections, pos_dict)

Human_pose_estimator = Human_pose_estimation()

plot_pos_x = []
plot_pos_y = []
plot_angle = []
hej = 0
counter = 0

# Initial target when starting the drone
target_marker = 0
Target = path_finder.calculate_target_pose(target_marker, 1.5, 1.4)
target_marker = 0

# Decide which PID to tune
pid_tune = drone.yaw_pid


pathing = 0


# Define the VideoWriter object
size = (960, 720)
FPS = 30
result = cv2.VideoWriter('filename.avi',  
                         cv2.VideoWriter_fourcc(*'MJPG'), 
                         FPS, size) 

avoidance = False
avoidance_state = 0
keyboard_mode = False
start_time = time.time()
avoidance_distance = 0

wtd_rot = [0, 0, 0]
wtd_trans = [0, 0, 0]
img = cv2.cvtColor(drone.camera.frame, cv2.COLOR_BGR2RGB)
i = 0
while True:
    cv2.waitKey(2)
    
    start_time = time.time()
    img_prev = img
    if run_drone:
        img = cv2.cvtColor(drone.camera.frame, cv2.COLOR_BGR2RGB)
        opt_cam_matrix, _ = cv2.getOptimalNewCameraMatrix(intrinsic_camera, distortion, (960, 720), 1, (960, 720))
        img = cv2.undistort(img, intrinsic_camera, distortion, None, opt_cam_matrix)
    else:
        ret, img = cap.read()
    
    
    if i < 5:
        temp = 0
        i = i+1
    else:
        temp = (img - img_prev).any()

    # Ensure that the pose estimation and human detection is only done if a new frames appears
    if temp:
        img2, wtd_rot, wtd_trans = pose_estimator.pose_estimation(img)
        img2 = Human_pose_estimator.get_pose(img2)
        
        # write to video
        result.write(img2)

    # Calculate the error between the drone and the target
    [drone.error_x, drone.error_y, drone.error_z], drone.error_yaw = pose_estimator.estimate_dTtarget(Target)
    drone.average_error()
    
    # Tune the PID parameters
    drone.yaw_pid.tune_pid()

    # Display the battery level
    drone.information_on_video(img, print_pid=False)

    # If the drone is in hover mode, handle the hover
    drone.hover()

    # Control the drone with the keyboard
    drone.keyboard_control()

    # Manually start the human avoidance, only used for testing
    if keyboard.is_pressed('i'):
        avoidance = True
        avoidance_state = 0
        drone.change_velocity(0, 20, 0, 0)
        drone.update_velocity()
        avoidance_distance = 0
    
    # State machine for the human avoidance
    if avoidance:
        print("avoid distance: ", avoidance_distance)
        if avoidance_state == 0:
            if Human_pose_estimator.person_to_close():
                direction = Human_pose_estimator.fly_right_or_left_around_person()
                if direction == "Right":
                    thread = threading.Thread(target=right_thread, args=(drone, int(20)))
                    thread.start()
                    avoidance_state = 1

                    # count left and right to get back on track
                    avoidance_distance = avoidance_distance + 1

                elif direction == "Left":
                    thread = threading.Thread(target=left_thread, args=(drone, int(20)))
                    thread.start()
                    avoidance_state = 1

                    # count left and right to get back on track
                    avoidance_distance = avoidance_distance - 1
                else:
                    drone.change_velocity(0, 20, 0, 0)
                    drone.update_velocity()
                
            else:
                drone.change_velocity(0, 20, 0, 0)
                drone.update_velocity()
        if avoidance_state == 1:
            if not thread.is_alive():
                if Human_pose_estimator.fly_right_or_left_around_person() == "all good":
                    avoidance_state = 2
                else:
                    avoidance_state = 0
        if avoidance_state == 2:
            thread = threading.Thread(target=forward_thread, args=(drone, int(200)))
            thread.start()
            avoidance_state = 3
        if avoidance_state == 3:
            if not thread.is_alive():
                avoidance_state = 4
        if avoidance_state == 4:
            if avoidance_distance > 0:
                thread = threading.Thread(target=left_thread, args=(drone, int(20*avoidance_distance)))
                thread.start()
                avoidance_state = 5
            else:
                thread = threading.Thread(target=right_thread, args=(drone, int(20*abs(avoidance_distance))))
                thread.start()
                avoidance_state = 5
        if avoidance_state == 5:
            if not thread.is_alive():
                avoidance_distance = 0
                avoidance_state = 0
                avoidance = False
                drone.change_velocity(0, 20, 0, 0)
                drone.update_velocity()

    # Manually stop the human avoidance, only used for testing
    if keyboard.is_pressed('o'):
        avoidance = False
        drone.change_velocity(0, 0, 0, 0)
        drone.update_velocity()
    
    # Start the pathing
    if keyboard.is_pressed('p'):
        while keyboard.is_pressed('p'):
            continue
        drone.is_pathing = True
        pathing = 1
        graph.dijkstra(12)
        target_marker = graph.get_next_node(target_marker)

    # state machine for the pathing
    if drone.is_pathing:
        if pathing == 1:
            #Target = [0., 2, 1.4, np.pi]
            drone.is_hovering = True
            if drone.average_error_close_to_zero():
                pathing = 2
                drone.is_hovering = False
                [target_x, target_y, target_z], target_yaw = pose_estimator.estimate_dTtarget(pos_dict[target_marker])
                new_jaw = path_finder.calculate_heading([target_x, target_y, target_z, target_yaw])
        elif pathing == 2:
            start_yaw = drone.get_internal_yaw()
            print(new_jaw)
            if new_jaw < 0:
                thread = threading.Thread(target=cw_thread, args=(drone, int(abs(new_jaw))))
                thread.start()
            else:
                thread = threading.Thread(target=ccw_thread, args=(drone, int(new_jaw)))
                thread.start()
            pathing = 3
        elif pathing == 3:
            if not thread.is_alive():
                end_yaw = drone.get_internal_yaw()
                if abs(start_yaw) < abs(end_yaw) + 0.1 and abs(start_yaw) > abs(end_yaw) - 0.1:
                    pathing = 2
                else:
                    pathing = 4
                    drone.change_velocity(0, 20, 0, 0)
                    drone.update_velocity()
        elif pathing == 4:
            if Human_pose_estimator.person_to_close():
                avoidance = True


            if path_finder.dist_to_target(wtd_trans, target_marker) < 2.0:
                avoidance = False
                avoidance_state = 0
                
                if target_marker in pose_estimator.ids:
                    pathing = 5
                    drone.change_velocity(0, 0, 0, 0)
                    drone.update_velocity()
                else:
                    drone.change_velocity(0, 0, 0, 25)
                    drone.update_velocity()
        elif pathing == 5:
            Target = path_finder.calculate_target_pose(target_marker, 1.5, 1.4)
            
            drone.is_hovering = True
            if drone.average_error_close_to_zero():
                pathing = 2
                target_marker = graph.get_next_node(target_marker)
                Target = path_finder.calculate_target_pose(target_marker, 1.5, 1.4)
                if target_marker == -1:
                    pathing = 6
                    break
                drone.is_hovering = False
                [target_x, target_y, target_z], target_yaw = pose_estimator.estimate_dTtarget(pos_dict[target_marker])
                new_jaw = path_finder.calculate_heading([target_x, target_y, target_z, target_yaw])
        elif pathing == 6:
            drone.stop()

    # Stop the drone
    if keyboard.is_pressed('esc'):
        break

    # Land the drone
    if keyboard.is_pressed('l') and drone.is_flying == True:
        drone.stop()
    
    # Launch the drone
    if keyboard.is_pressed('t') and drone.is_flying == False:
        drone.launch()      

    # Change the target marker
    if keyboard.is_pressed('1'):
        while keyboard.is_pressed('1'):
            continue
        target_marker = 0
        Target = path_finder.calculate_target_pose(0, 1.5, 1.4)
    
    if keyboard.is_pressed('2'):
        while keyboard.is_pressed('2'):
            continue
        Target = path_finder.calculate_target_pose(0, 1.5, 1.4)

    if keyboard.is_pressed('3'):
        while keyboard.is_pressed('3'):
            continue
        thread = threading.Thread(target=cw_thread, args=(drone, int(abs(5))))
        thread.start()


# Release the VideoWriter object
result.release()

