"""
AR Drone 2 wrapper class
extending the ARDrone class from pyardrone

@author: elie
"""

from pyardrone import ARDrone, at

import time
import numpy as np
import threading

# command: roll (rad), pitch (rad), vertical_speed(mm/s -> m/s), yaw (rad/s)
# TODO: get current values from UAV
DEFAULT_MAX_VALUES = np.array([2.0943999e-01,2.0943999e-01,7.0000000e+02*1e-3,1.7278759e+00]) 
LIMIT_MAX_VALUES = np.array([0.52,0.52,2000*1e-3,6.11])

class ARDroneController(ARDrone):
    
    def __init__(self):
        super().__init__()
        self.landing = False
        self.takingoff = False
        self.controlling = False
        self.command = np.zeros(4)
        self.max_values = LIMIT_MAX_VALUES
    
    def trim(self):
        """ Sends the trim command. """
        self.send(at.FTRIM())
        
    def set_command_max(self, roll_pitch, verical_speed, yaw):
        if np.greater(np.array([roll_pitch, verical_speed, yaw]),LIMIT_MAX_VALUES[1:]).any():
            print("Max values not applied because unsafe.")
        else:
            self.send(at.CONFIG('control:euler_angle_max', roll_pitch))
            self.send(at.CONFIG('control:control_vz_max', verical_speed*1e3)) # m/s -> mm/s
            self.send(at.CONFIG('control:control_yaw', yaw)) # TODO: deg vs rad
            self.max_values = np.array([roll_pitch, roll_pitch, verical_speed, yaw])
        
    def normalize_command(self, command):
        command_norm = command/self.max_values
        command_norm = np.maximum(command_norm,-1)
        command_norm = np.minimum(command_norm,1)
        return command_norm
    
    def land_wait(self):
        while self.state.fly_mask:
            self.land()
        self.landing = False
            
    def takeoff_wait(self):
        while not self.state.fly_mask:
            self.takeoff()
        self.takingoff = False
    
    def _continuous_send(self, sleep_time):
        while True:
            if self.landing:
                self.land_wait()
            elif self.takingoff:
                self.takeoff_wait()
            if self.controlling:
                self._move(*self.command)
            else:
                self.hover()
            time.sleep(sleep_time)
            
    def send_control(self):
        if self.landing:
            if self.state.fly_mask:
                self.land()
            else:
                self.landing = False
        elif self.takingoff:
            if not self.state.fly_mask:
                self.takeoff()
            else:
                self.takingoff = False
        if self.controlling:
            self._move(*self.command)
        else:
            self.hover()