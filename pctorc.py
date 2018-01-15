"""
Send UDP packet to PCtoRC for RC Logger Xtreme

@author: elie
"""

import socket
import time
import numpy as np
import math

import threading

BEGINNER_MODE = 0
SPORT_MODE = 1
EXPERT_MODE = 2

ACROBATIC_MODE = 0
NORMAL_FLIGHT = 1
ALTITUDE_HEIGHT_HOLD = 2

SWITCH_DEFAULTS = [SPORT_MODE,NORMAL_FLIGHT,0]

IP = '127.0.0.1'
PORTS = [25001,25002,25003,25004,25005,25006,25007]
N_STICK = 4
N_SWITCH = 3

# command format:
# throttle, rudder (yaw/psi), elevator (pitch/theta), aileron (roll/phi), ...


BASE_VALUES = np.array([[0,6000,6000,6000]]).T
MAX_VALUES = np.array([[12000,12000,12000,12000]]).T
MIN_VALUES = np.array([[0,0,0,0]]).T

SWITCH_MOTORS_SIGNAL = np.array([[0,0,0,0],
                                 [6000,12000,12000,6000],
                                 [6000,6000,6000,6000],
                                 [6000,6000,6000,6000]])

# thrust (calculated): 2553
# thrust (tested): 2170
gains = np.array([[2200,1,(-50*180)/math.pi,(-50*180)/math.pi]]).T

class UAVController:
    def __init__(self, client_ip=IP, client_ports=PORTS):
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.client_ip = client_ip
        self.client_ports = client_ports
        
        self.motors_started = False
        
        self.reset()
        
        self._send_thread = threading.Thread(group=None, target=self._continuous_send, name="PCtoRC Sender Thread", args=(0.025, ))
        self._send_thread.daemon = True
        self._send_thread.start()
        
    def send(self, value, port_i):
        self.sock.sendto(np.uint16(value), (self.client_ip,self.client_ports[port_i]))

    def send_command(self, command_ppm):
        for i in range(N_STICK):
            self.send(command_ppm[i], i)
            
    def set_flight_mode(self, mode):
        self.send(mode, 4) 
        
    def reset(self):
        self.command_ppm = BASE_VALUES
        for i in range(N_STICK):
            self.send(BASE_VALUES[i], i)
        for i in range(N_SWITCH):
            self.send(SWITCH_DEFAULTS[i], N_STICK+i)
            
    def convert_to_ppm(self,command):
        command_ppm = command*gains + BASE_VALUES
        command_ppm = np.maximum(command_ppm,MIN_VALUES)
        command_ppm = np.minimum(command_ppm,MAX_VALUES)
        return command_ppm

    def start_motors(self):
        if not self.motors_started:
            self.command_ppm = SWITCH_MOTORS_SIGNAL[:,1]
            time.sleep(2)
            self.command_ppm = SWITCH_MOTORS_SIGNAL[:,0]
            self.motors_started = True
        else:
            print("Motors already running.")
            
    def stop_motors(self):
        if self.motors_started:
            self.command_ppm = SWITCH_MOTORS_SIGNAL[:,1]
            time.sleep(2)
            self.command_ppm = SWITCH_MOTORS_SIGNAL[:,0]
            self.motors_started = False
        else:
            print("Motors already stopped.")
            
    def switch_motors(self):
        self.command_ppm = SWITCH_MOTORS_SIGNAL[:,1]
        time.sleep(2)
        self.command_ppm = SWITCH_MOTORS_SIGNAL[:,0]
        self.motors_started = not self.motors_started
        
    def _continuous_send(self, sleep_time):
        while True:
            time.sleep(sleep_time)
            self.send_command(self.command_ppm)