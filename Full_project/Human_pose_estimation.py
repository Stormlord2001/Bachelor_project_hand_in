import cv2
import mediapipe as mp
import numpy as np
import math

#Tutorial
# https://www.youtube.com/watch?v=brwgBf6VB0I

class Human_pose_estimation():
    def __init__(self):
        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose()

        self.focal_length = 910

        self.dist = 0
        self.human_center = [0,0]

    # Returns the frame with the pose estimation
    def get_pose(self, frame):
        framergb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.pose.process(framergb)

        if result.pose_landmarks:
            Torso = []

            for id, lm in enumerate(result.pose_landmarks.landmark):
                h, w, c = frame.shape
                cx, cy = int(lm.x*w), int(lm.y*h)
                
                # Draw the landmarks
                cv2.circle(frame, (cx, cy), 10, (255, 0, 0), cv2.FILLED)
                cv2.putText(frame, f'{lm.z :.2f}', (cx, cy), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

                # Get the landmarks of the torso
                if 11 <= id <= 12 or 23 <= id <= 24:
                    Torso.append((cx, cy, lm.z*w/2))
            
            pixel_dist = (math.dist(Torso[0], Torso[2])+math.dist(Torso[1], Torso[3]))/2
            
            # Calculate the distance of the person to the camera
            self.dist = 55 * self.focal_length / pixel_dist
            self.human_center = (int((Torso[0][0] + Torso[1][0] + Torso[2][0] + Torso[3][0]) / 4), int((Torso[0][1] + Torso[1][1] + Torso[2][1] + Torso[3][1]) / 4))
            
            # Draw the human center
            cv2.circle(frame, (self.human_center[0], self.human_center[1]), 10, (0, 255, 0), cv2.FILLED)
        else:
            self.dist = None
            self.human_center = [0,0]
        return frame

    # Returns the distance of the person to the camera
    def get_distance(self):
        return self.dist
    
    # Check if the person is to close to the camera
    def person_to_close(self):
        if self.dist == None:
            return False
        else:
            return self.dist < 150
    
    # Return "Right" or "Left" if the person is to close to the camera
    def fly_right_or_left_around_person(self):
        if not self.person_to_close():
            return "all good"
        if 0+self.dist <= self.human_center[0] <= 480:
            return "Right"
        if 480 <= self.human_center[0] <= 960-self.dist:
            return "Left"
        else:
            return "all good"
