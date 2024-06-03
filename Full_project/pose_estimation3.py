import cv2
import numpy as np

from scipy.spatial.transform import Rotation
from filterpy.kalman import KalmanFilter

class PoseEstimation3():
    def __init__(self, pos_dict, dictionary, parameters, camera_matrix, distortion_vector, object_points, markers_on_paper, how_many_markers, angle = 0, L = 2, sigma_z_sq = 2, sigma_a_sq = 100, use_kalman = False, second_order_kalman = False, adaptive_kalman = False, use_specific_marker = None):
        self.pos_dict = pos_dict
        self.dictionary = dictionary
        self.parameters = parameters
        self.camera_matrix = camera_matrix
        self.distortion_vector = distortion_vector
        

        self.prev_translation = [0,0,0]
        self.prev_rotation = [0,0,0]

        self.dcorrectTw = np.zeros((4,4))
        self.wTdcorrect = np.zeros((4,4))

        self.object_points = object_points
        self.markers_on_paper = markers_on_paper
        self.how_many_markers = how_many_markers
        self.angle = angle
        self.use_specific_marker = use_specific_marker

        self.ids = []

        self.vx = 0.
        self.vy = 0.
        self.vz = 0.

        self.L = L
        self.sigma_z_sq = sigma_z_sq
        self.sigma_a_sq = sigma_a_sq

        self.kalman_enabled = False
        self.second_order = second_order_kalman
        self.adaptive = adaptive_kalman
        self.fx_count = 0
        self.fy_count = 0
        self.fz_count = 0

        if use_kalman:
            self.kalman_enabled = True

            dt = 1/30
            sigma_z_sq = self.sigma_z_sq #2
            sigma_a_sq = self.sigma_a_sq #100
            L = self.L#2

            if self.second_order:
                self.fx = KalmanFilter(dim_x = 3, dim_z = 1)
                self.fx.x = np.array([[0.], [0.], [0.]])
                self.fx.F = np.array([[1., dt, 1/2*dt**2],[0., 1., dt],[0., 0., 1.]])
                self.fx.H = np.array([[1., 0., 0.]])
                self.fx.P *= L             # Good values (0.1)
                self.fx.R = sigma_z_sq    # Good values (0.01)
                self.fx.Q = [[(dt**5)/20*sigma_a_sq, (dt**4)/8*sigma_a_sq, (dt**3)/6*sigma_a_sq],[(dt**4)/8*sigma_a_sq,(dt**3)/3*sigma_a_sq, (dt**2)/2*sigma_a_sq],[(dt**3)/6*sigma_a_sq,(dt**2)/2*sigma_a_sq, dt*sigma_a_sq]]
            
                self.fy = KalmanFilter(dim_x = 3, dim_z = 1)
                self.fy.x = np.array([[0.], [0.], [0.]])
                self.fy.F = np.array([[1., dt, 1/2*dt**2],[0., 1., dt],[0., 0., 1.]])
                self.fy.H = np.array([[1., 0., 0.]])
                self.fy.P *= L             # Good values (0.1)
                self.fy.R = sigma_z_sq    # Good values (0.01)
                self.fy.Q = [[(dt**5)/20*sigma_a_sq, (dt**4)/8*sigma_a_sq, (dt**3)/6*sigma_a_sq],[(dt**4)/8*sigma_a_sq,(dt**3)/3*sigma_a_sq, (dt**2)/2*sigma_a_sq],[(dt**3)/6*sigma_a_sq,(dt**2)/2*sigma_a_sq, dt*sigma_a_sq]]
                self.fy.Q = [[1,0,0],[0,1,0],[0,0,1]]

                self.fz = KalmanFilter(dim_x = 3, dim_z = 1)
                self.fz.x = np.array([[0.], [0.], [0.]])
                self.fz.F = np.array([[1., dt, 1/2*dt**2],[0., 1., dt],[0., 0., 1.]])
                self.fz.H = np.array([[1., 0., 0.]])
                self.fz.P *= L             # Good values (0.1)
                self.fz.R = sigma_z_sq    # Good values (0.01)
                self.fz.Q = [[(dt**5)/20*sigma_a_sq, (dt**4)/8*sigma_a_sq, (dt**3)/6*sigma_a_sq],[(dt**4)/8*sigma_a_sq,(dt**3)/3*sigma_a_sq, (dt**2)/2*sigma_a_sq],[(dt**3)/6*sigma_a_sq,(dt**2)/2*sigma_a_sq, dt*sigma_a_sq]]
                self.fz.Q = [[1,0,0],[0,1,0],[0,0,1]]
            
            else:
                self.fx = KalmanFilter(dim_x = 2, dim_z = 1)
                self.fx.x = np.array([[0.], [0.]])
                self.fx.F = np.array([[1., dt],[0., 1.]])
                self.fx.H = np.array([[1., 0.]])
                self.fx.P *= L             # Good values (0.1)
                self.fx.R = sigma_z_sq    # Good values (0.01)
                self.fx.Q = [[(dt**4)/4*sigma_a_sq, (dt**3)/2*sigma_a_sq],[(dt**3)/2*sigma_a_sq, dt**2*sigma_a_sq]]
                #self.fx.Q = [[1,0],[0,1]]

                self.fy = KalmanFilter(dim_x = 2, dim_z = 1)
                self.fy.x = np.array([[0.], [0.]])
                self.fy.F = np.array([[1., dt],[0., 1.]])
                self.fy.H = np.array([[1., 0.]])
                self.fy.P *= L             # Good values (0.1)
                self.fy.R = sigma_z_sq     # Good values (0.01)
                self.fy.Q = [[(dt**4)/4*sigma_a_sq, (dt**3)/2*sigma_a_sq],[(dt**3)/2*sigma_a_sq, dt**2*sigma_a_sq]]
                #self.fx.Q = [[1,0],[0,1]]

                self.fz = KalmanFilter(dim_x = 2, dim_z = 1)
                self.fz.x = np.array([[0.], [0.]])
                self.fz.F = np.array([[1., dt],[0., 1.]])
                self.fz.H = np.array([[1., 0.]])
                self.fz.P *= L             # Good values (0.1)
                self.fz.R = sigma_z_sq     # Good values (0.01)
                self.fz.Q = [[(dt**4)/4*sigma_a_sq, (dt**3)/2*sigma_a_sq],[(dt**3)/2*sigma_a_sq, dt**2*sigma_a_sq]]
                #self.fx.Q = [[1,0],[0,1]]


    
    def aruco_display(self, corners, ids, image):
        if (len(corners) > 0):

            ids = ids.flatten()

            for (markerCorner, markerID) in zip(corners, ids):

                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))

                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

                cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)
            
        return image

    def correct_markers_seen(self, bucket):
        for i in range(0, self.markers_on_paper):
            if bucket[i][0][0] == 0:
                return False
        return True
    
    def combine_corners(self, corners):
        corners_combined = corners[0]
        for i in range(1, len(corners)):
            corners_combined = np.append(corners_combined, corners[i], axis=0)
        return np.float64(corners_combined)

    def pose_estimation(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary=self.dictionary, parameters=self.parameters)
        t1 = [0, 0, 0]
        t2 = [0, 0, 0]
        real_translation = [0,0,0]
        real_rotation = [0,0,0]

        dcorrectTw_list = []
        wTdcorrect_list = []

        yaw = 0
        id_list = []

        buckets = np.full((50, 4, 2), 0, dtype=float)
        if (len(corners) > 1):
            for i in range(len(ids)):
                buckets[ids[i][0]] = corners[i][0]

        if (len(corners) > 1):
            for i in range(0,len(buckets),self.markers_on_paper):
                if self.correct_markers_seen(buckets[i:i+self.markers_on_paper]):
                    if self.use_specific_marker is not None:
                        if not (self.use_specific_marker == i):
                            continue
                    id_list.append(i)

                    corners_combined = self.combine_corners(buckets[i:i+self.how_many_markers])
                    ret = cv2.solvePnPGeneric(self.object_points, corners_combined, self.camera_matrix, self.distortion_vector, flags=cv2.SOLVEPNP_IPPE)
                    tvec1 = np.array(ret[2][0]).reshape(3)
                    rvec1 = ret[1][0]
                    rmat1, _ = cv2.Rodrigues(rvec1)

                    tvec2 = np.array(ret[2][1]).reshape(3)
                    rvec2 = ret[1][1]
                    rmat2, _ = cv2.Rodrigues(rvec2)
                    
                    t1 = np.transpose(-rmat1).dot(tvec1)
                    t2 = np.transpose(-rmat2).dot(tvec2)

                    dTm_transformation = np.zeros((4,4))

                    if t1[1] > 0:
                        real_translation = tvec1
                        real_rotation = rvec1
                        real_rotation_matrix = rmat1
                        
                    elif t2[1] > 0:
                        real_translation = tvec2
                        real_rotation = rvec2
                        real_rotation_matrix = rmat2
                    else:
                        continue
                    dTm_transformation[0:3,0:3] = real_rotation_matrix
                    dTm_transformation[0:3,3] = real_translation
                    dTm_transformation[3,3] = 1

                    dcorrectTd_transformation = [[1,0,0,0],[0,-0.3826834,0.9238795,0],[0,-0.9238795,-0.3826834,0],[0,0,0,1]]

                    angle = np.deg2rad(self.angle)
                    test = [[1,0,0,0],[0,np.cos(angle),-np.sin(angle),0],[0,np.sin(angle),np.cos(angle),0],[0,0,0,1]]
                    mTmcorrect_transformation = [[-1,0,0,0],[0,0,1,0],[0,1,0,0],[0,0,0,1]] 

                    mcorrectTw_transformation = [[np.cos(self.pos_dict[i][3]),np.sin(self.pos_dict[i][3]),0,-(self.pos_dict[i][0]*np.cos(self.pos_dict[i][3])+self.pos_dict[i][1]*np.sin(self.pos_dict[i][3]))],
                                                 [-np.sin(self.pos_dict[i][3]),np.cos(self.pos_dict[i][3]),0,-(self.pos_dict[i][1]*np.cos(self.pos_dict[i][3])-self.pos_dict[i][0]*np.sin(self.pos_dict[i][3]))],
                                                 [0,0,1,-self.pos_dict[i][2]],[0,0,0,1]]

                    dcorrectTw = dcorrectTd_transformation @ dTm_transformation @ mTmcorrect_transformation @ test @ mcorrectTw_transformation

                    wTdcorrect = np.zeros((4,4))
                    wTdcorrect[0:3, 0:3] = np.transpose(dcorrectTw[0:3, 0:3])
                    wTdcorrect[0:3, 3] = -np.transpose(dcorrectTw[0:3, 0:3]).dot(dcorrectTw[0:3, 3])
                    wTdcorrect[3, 3] = 1   

                    dcorrectTw_list.append(dcorrectTw)
                    wTdcorrect_list.append(wTdcorrect)              

                    # Draw axis and append results
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.distortion_vector, real_rotation, real_translation, 0.04)

                else:
                    continue
            
            # Save transformation matrices to global variables 
            if len(dcorrectTw_list) > 0:
                self.dcorrectTw = np.mean(dcorrectTw_list, axis=0)
                self.wTdcorrect = np.mean(wTdcorrect_list, axis=0)
                real_translation = self.wTdcorrect[0:3,3]

                if self.kalman_enabled:
                    self.fx.predict()
                    self.fy.predict()
                    self.fz.predict()

                    if self.adaptive:
                        epsilon_x = self.fx.y * 1/self.fx.S * self.fx.y
                        epsilon_y = self.fy.y * 1/self.fy.S * self.fy.y
                        epsilon_z = self.fz.y * 1/self.fz.S * self.fz.y

                        if epsilon_x > 0.001:
                            self.fx_count += 1  
                            self.fx.Q[0][0] =  self.fx.Q[0][0] * 5
                            self.fx.Q[0][1] =  self.fx.Q[0][1] * 5
                            self.fx.Q[1][0] =  self.fx.Q[1][0] * 5
                            self.fx.Q[1][1] =  self.fx.Q[1][1] * 5
                        elif self.fx_count > 0:
                            self.fx_count -= 1
                            self.fx.Q[0][0] =  self.fx.Q[0][0] / 5
                            self.fx.Q[0][1] =  self.fx.Q[0][1] / 5
                            self.fx.Q[1][0] =  self.fx.Q[1][0] / 5
                            self.fx.Q[1][1] =  self.fx.Q[1][1] / 5
               
                        if epsilon_y > 0.001:
                            self.fy_count += 1  
                            self.fy.Q[0][0] =  self.fy.Q[0][0] * 5
                            self.fy.Q[0][1] =  self.fy.Q[0][1] * 5
                            self.fy.Q[1][0] =  self.fy.Q[1][0] * 5
                            self.fy.Q[1][1] =  self.fy.Q[1][1] * 5
                        elif self.fy_count > 0:
                            self.fy_count -= 1
                            self.fy.Q[0][0] =  self.fy.Q[0][0] / 5
                            self.fy.Q[0][1] =  self.fy.Q[0][1] / 5
                            self.fy.Q[1][0] =  self.fy.Q[1][0] / 5
                            self.fy.Q[1][1] =  self.fy.Q[1][1] / 5
                        
                        if epsilon_z > 0.001:
                            self.fz_count += 1  
                            self.fz.Q[0][0] =  self.fz.Q[0][0] * 5
                            self.fz.Q[0][1] =  self.fz.Q[0][1] * 5
                            self.fz.Q[1][0] =  self.fz.Q[1][0] * 5
                            self.fz.Q[1][1] =  self.fz.Q[1][1] * 5
                        elif self.fz_count > 0:
                            self.fz_count -= 1
                            self.fz.Q[0][0] =  self.fz.Q[0][0] / 5
                            self.fz.Q[0][1] =  self.fz.Q[0][1] / 5
                            self.fz.Q[1][0] =  self.fz.Q[1][0] / 5
                            self.fz.Q[1][1] =  self.fz.Q[1][1] / 5
                            
                    self.fx.update(self.dcorrectTw[0,3])
                    self.fy.update(self.dcorrectTw[1,3])
                    self.fz.update(self.dcorrectTw[2,3])

                    new_dcorrectTw_translation = np.array([self.fx.x[0], self.fy.x[0], self.fz.x[0]]).flatten()
                    self.dcorrectTw[0:3,3] = new_dcorrectTw_translation
                    real_translation = -np.transpose(self.dcorrectTw[0:3, 0:3]).dot(self.dcorrectTw[0:3, 3])

                r = Rotation.from_matrix(self.wTdcorrect[0:3,0:3])
                real_rotation = r.as_euler('xyz', degrees=False)
            else:
                if self.kalman_enabled:
                    self.fx.predict()
                    self.fy.predict()
                    self.fz.predict()

                    new_dcorrectTw_translation = np.array([self.fx.x[0], self.fy.x[0], self.fz.x[0]]).flatten()
                    self.dcorrectTw[0:3,3] = new_dcorrectTw_translation
                    real_translation = -np.transpose(self.dcorrectTw[0:3, 0:3]).dot(self.dcorrectTw[0:3, 3])

        else:
            if self.kalman_enabled:
                self.fx.predict()
                self.fy.predict()
                self.fz.predict()

                new_dcorrectTw_translation = np.array([self.fx.x[0], self.fy.x[0], self.fz.x[0]]).flatten()
                self.dcorrectTw[0:3,3] = new_dcorrectTw_translation
                real_translation = -np.transpose(self.dcorrectTw[0:3, 0:3]).dot(self.dcorrectTw[0:3, 3])

        
        self.ids = id_list

        
        self.aruco_display(corners, ids, frame)
        return frame, real_rotation, real_translation
    
    def estimate_dTtarget(self, target):
        dcorrectTtarget = np.zeros((4,4))
        wTtarget = [[np.cos(target[3]),-np.sin(target[3]),0,target[0]],
                    [np.sin(target[3]),np.cos(target[3]),0,target[1]],
                    [0,0,1,target[2]],[0,0,0,1]]

        dcorrectTtarget = self.dcorrectTw @ wTtarget

        translation = dcorrectTtarget[0:3,3]

        r = Rotation.from_matrix(dcorrectTtarget[0:3,0:3])
        yaw = r.as_euler('xyz', degrees=False)[2]

        return translation, yaw
    
    def update_velocity(self, velocities):
        self.vx = velocities[0] / 3000 
        self.vy = -velocities[1] / 3000 
        self.vz = velocities[2] / 3000

