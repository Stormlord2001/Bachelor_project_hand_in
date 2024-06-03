import cv2
import numpy as np
import djitellopy as Tello
import keyboard

from pid import *

#Middle pixel of drone camera is located at 480, 170
#box_vertical = top:30 bottom:240
#box_horizontal = right:180 left:780

class Drone():
    # All variables and methods written for the tello drone
    def __init__(self):
        self.tello = Tello.Tello()

        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 40

        self.x_pos = 0
        self.y_pos = 0
        self.z_pos = 0
        self.yaw = 0

        self.x_pid = pid(0.96, 2.247/20, 0.562/5, print_debug=False)
        self.y_pid = pid(0.96, 2.247/20, 0.562/5)
        self.z_pid = pid(1.06, 0.21235, 0.562/5)
        self.yaw_pid = pid(0.96*3, 2.247/20, 0.562/5, True)

        self.camera = None  

        self.send_rc_control = False
        self.is_flying = False
        self.is_hovering = False
        self.is_pathing = False

        # Connect to drone
        self.tello.connect()
        self.tello.set_speed(self.speed)

        # Get battery level
        self.battery_level = self.tello.get_battery()

        # Drone height
        self.init_height = self.tello.get_barometer()

        self.error_x = 0
        self.error_y = 0
        self.error_z = 0
        self.error_yaw = 0
        self.window_error_x = np.zeros(5).tolist()
        self.window_error_y = np.zeros(5).tolist()
        self.window_error_z = np.zeros(5).tolist()
        self.window_error_yaw = np.zeros(5).tolist()
        self.avg_error_x = 0
        self.avg_error_y = 0
        self.avg_error_z = 0
        self.avg_error_yaw = 0

        self.keyboard_mode = False


        self.speed_lim_first = 20
        self.count_speed = 0
        

    def camera_on(self):
        # In case streaming is on. This happens when we quit
        # the program without the escape key
        self.tello.streamoff()
        self.tello.streamon()

        # Object to acces camera
        self.camera = self.tello.get_frame_read()

    def camera_off(self):
        self.tello.streamoff()
        self.camera = None

    def launch(self):
        # Enable control of drone
        self.tello.takeoff()
        self.is_flying = True
        self.send_rc_control = True
    
    def stop(self):
        self.change_velocity(0, 0, 0, 0)
        self.update_velocity()
        self.tello.land()
        # Disable control of drone
        self.is_flying = False
        self.is_hovering = False
        self.is_pathing = False
        self.send_rc_control = False

    def average_error(self):
        self.window_error_x.pop(0)
        self.window_error_x.append(self.error_x)
        self.window_error_y.pop(0)
        self.window_error_y.append(self.error_y)
        self.window_error_z.pop(0)
        self.window_error_z.append(self.error_z)
        self.window_error_yaw.pop(0)
        self.window_error_yaw.append(self.error_yaw)

        self.avg_error_x = np.mean(self.window_error_x)
        self.avg_error_y = np.mean(self.window_error_y)
        self.avg_error_z = np.mean(self.window_error_z)
        self.avg_error_yaw = np.mean(self.window_error_yaw)

    def average_error_close_to_zero(self):
        if abs(self.error_x) < 0.15 and abs(self.error_y) < 0.15 and abs(self.error_z) < 0.15 and abs(self.error_yaw) < 0.15:
            return True


    def change_velocity(self, lr, fb, ud, yaw):
        self.left_right_velocity = lr
        self.for_back_velocity = fb
        self.up_down_velocity = ud
        self.yaw_velocity = yaw
    
    def hover(self):
        if keyboard.is_pressed('q'):
            while(keyboard.is_pressed('q')):
                continue
            
            if self.is_hovering == False:
                self.is_hovering = True
                self.x_pid.reset_error()
                self.y_pid.reset_error()
                self.z_pid.reset_error()
                self.yaw_pid.reset_error()

            else:
                self.is_hovering = False
                self.change_velocity(0, 0, 0, 0)
                self.update_velocity()
        
        if self.is_hovering:
            if self.count_speed < 100:
                self.x_pid.speed = self.speed_lim_first
                self.y_pid.speed = self.speed_lim_first
                self.z_pid.speed = self.speed_lim_first
                self.yaw_pid.speed = 30
                self.count_speed += 1
            else:
                self.x_pid.speed = self.speed
                self.y_pid.speed = self.speed
                self.z_pid.speed = self.speed
                self.yaw_pid.speed = self.speed
            self.send_rc_control = 1

            if self.error_x != 0 and self.error_y != 0 and self.error_z != 0:
                self.fly_to_target()
            else:
                self.change_velocity(0, 0, 0, 0)
            self.update_velocity()
        else:
            self.x_pid.reset_error()
            self.y_pid.reset_error()
            self.z_pid.reset_error()
            self.yaw_pid.reset_error()
            self.count_speed = 0

    def get_internal_yaw(self):
        return self.tello.get_yaw()*np.pi/180

    def fly_to_target(self):
        self.change_velocity(self.x_pid.calculate(self.avg_error_x),
                             self.y_pid.calculate(self.avg_error_y),
                             self.z_pid.calculate(self.avg_error_z),
                             -self.yaw_pid.calculate(self.avg_error_yaw))

        #print(f'x: {self.avg_x: 5}\t y: {self.avg_y: 5}\t z: {self.avg_z: 5}\t yaw: {self.yaw}')

    def update_velocity(self):
        if self.send_rc_control:
            self.tello.send_rc_control(int(self.left_right_velocity), int(self.for_back_velocity), int(self.up_down_velocity), int(self.yaw_velocity))
            self.battery_level = self.tello.get_battery()

    def get_velocity(self):
        return (self.left_right_velocity, self.for_back_velocity, self.up_down_velocity)

    def keyboard_control(self):
        if self.keyboard_mode:
            speed = 100
            if keyboard.is_pressed('up'):
                self.change_velocity(0, speed, 0, 0)
            
            if keyboard.is_pressed('down'):
                self.change_velocity(0, -speed, 0, 0)

            if keyboard.is_pressed('left'):
                self.change_velocity(-speed, 0, 0, 0)
            
            if keyboard.is_pressed('right'):
                self.change_velocity(speed, 0, 0, 0)

            if keyboard.is_pressed('w'):
                self.change_velocity(0, 0, speed, 0)
            
            if keyboard.is_pressed('s'):
                self.change_velocity(0, 0, -speed, 0)
            
            if keyboard.is_pressed('a'):
                self.change_velocity(0, 0, 0, -speed)

            if keyboard.is_pressed('d'):
                self.change_velocity(0, 0, 0, speed)
            self.update_velocity()

            if not keyboard.is_pressed('up') and not keyboard.is_pressed('down') and not keyboard.is_pressed('left') and not keyboard.is_pressed('right') and not keyboard.is_pressed('w') and not keyboard.is_pressed('s') and not keyboard.is_pressed('a') and not keyboard.is_pressed('d'):
                self.change_velocity(0, 0, 0, 0)
                self.update_velocity()

            if keyboard.is_pressed('k'):
                while keyboard.is_pressed('k'):
                    continue
                self.keyboard_mode = False
        else:
            if keyboard.is_pressed('k'):
                while keyboard.is_pressed('k'):
                    continue
                self.keyboard_mode = True
                self.change_velocity(0, 0, 0, 0)
                self.update_velocity()


    def information_on_video(self, img, print_pid = False):
        if print_pid:
            cv2.putText(img, "x_pid: {}, {}, {}".format(self.x_pid.kp, self.x_pid.ki, self.x_pid.kd), (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(img, "y_pid: {}, {}, {}".format(self.y_pid.kp, self.y_pid.ki, self.y_pid.kd), (5, 55), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(img, "z_pid: {}, {}, {}".format(self.z_pid.kp, self.z_pid.ki, self.z_pid.kd), (5, 85), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(img, "yaw_pid: {}, {}, {}".format(self.yaw_pid.kp, self.yaw_pid.ki, self.yaw_pid.kd), (5, 115), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(img, "Battery: {}".format(self.battery_level), (5, 720 - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("stream", img)