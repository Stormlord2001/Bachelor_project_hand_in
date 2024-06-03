import numpy as np
import time
import keyboard

class pid():
    def __init__(self, kp, ki, kd, rotation_pid=False, print_debug=False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.rotation_pid = rotation_pid
        self.prev_time = 0
        self.time = 0
        self.dt = 0.05

        self.error = 0
        self.prev_error = 0
        self.set_point = 0
        self.integral_value = 0

        self.is_saturated = False
        self.saturated_output = 0
        self.print_debug = print_debug
        self.speed = 50
    
    def calculate(self, error):
        self.prev_time = self.time
        self.time = time.time()
        if self.prev_time != 0:
            self.dt = self.time - self.prev_time
        else:
            self.prev_error = error
        self.error = error

        if self.rotation_pid:
            if self.error > np.pi:
                self.error = self.error - np.pi*2
            elif self.error < -np.pi:
                self.error = self.error + np.pi*2

        # Integral term
        if self.is_saturated:
            self.integral_value = self.integral_value
        else:
            self.integral_value = self.integral_value + self.error*self.dt

        # Derivative term
        self.derivative_value = (self.error - self.prev_error)/self.dt

        output = self.kp * self.error + self.ki * self.integral_value + self.kd * self.derivative_value
        self.prev_error = self.error

        # Anti wind-up
        if output < -1:
            self.saturated_output = -self.speed
            self.is_saturated = True
        elif output > 1:
            self.saturated_output = self.speed
            self.is_saturated = True
        else:
            self.saturated_output = output * self.speed
            self.is_saturated = False

        # output = int(np.tanh(output)*100)
        if self.print_debug:
            print(f'dt: {self.dt:.3f}\t error: {self.error:.2f}\t integral: {self.integral_value:.2f}\t derivative: {self.derivative_value:.2f}\t output: {self.saturated_output:.2f}')
        return self.saturated_output
    
    # Helper function to change the PID parameters
    def change_parameter(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    # Used to reset the integral value
    def reset_error(self):
        self.integral_value = 0

    # Used to tune the PID parameters
    def tune_pid(self):
        if keyboard.is_pressed('g'):
            while(keyboard.is_pressed('g')):
                    continue
            self.change_parameter(self.kp+0.1, self.ki, self.kd)

        if keyboard.is_pressed('b'):
            while(keyboard.is_pressed('b')):
                    continue
            self.change_parameter(self.kp-0.1, self.ki, self.kd)

        if keyboard.is_pressed('h'):
            while(keyboard.is_pressed('h')):
                    continue
            self.change_parameter(self.kp, self.ki+0.1, self.kd)

        if keyboard.is_pressed('n'):
            while(keyboard.is_pressed('n')):
                    continue
            self.change_parameter(self.kp, self.ki-0.1, self.kd)

        if keyboard.is_pressed('j'):
            while(keyboard.is_pressed('j')):
                    continue
            self.change_parameter(self.kp, self.ki, self.kd+0.05)

        if keyboard.is_pressed('m'):
            while(keyboard.is_pressed('m')):
                    continue
            self.change_parameter(self.kp, self.ki, self.kd-0.05)