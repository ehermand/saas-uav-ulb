"""
Simple Control GUI

@author: elie
"""

import tkinter as tk
from multiprocessing import Queue
import threading
import time
from enum import Enum

import numpy as np

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
    
class GuiControls(Enum):
    ON_OFF = 1
    EMERGENCY = 2
    CONTROL = 3
    RECORD = 5
    QUIT = 6

class Application(tk.Frame):
    def __init__(self, q_controls, q_gains, q_ref, q_data, master=None):
        super().__init__(master)
        self.pack()
        self.master = master
        
        self.q_controls = q_controls
        self.q_gains = q_gains
        self.q_ref = q_ref
        self.q_data = q_data
        
        self.ref_applied = False
        self.should_quit = False
        self.is_anim_paused = False
        
        # initialize plots
        self.f = Figure(figsize=(6,4), dpi=100)
        self.a1 = self.f.add_subplot(311)
        self.a2 = self.f.add_subplot(312,sharex=self.a1)
        self.a3 = self.f.add_subplot(313,sharex=self.a1)
        self.data = np.zeros([0,18])
        
        self.create_widgets()
        self._continuous_update_plot()

    def create_widgets(self):
        # create buttons
        self.fr_buttons = tk.Frame()
        
        self.bt_switch_motors = tk.Button(self.fr_buttons)
        self.bt_switch_motors["text"] = "Motors On/Off"
        self.bt_switch_motors["command"] = self.motors_switch
        
        self.bt_switch_control = tk.Button(self.fr_buttons)
        self.bt_switch_control["text"] = "Control On/Off"
        self.bt_switch_control["command"] = self.control_switch
        
        self.bt_switch_recording = tk.Button(self.fr_buttons)
        self.bt_switch_recording["text"] = "REC"
        self.bt_switch_recording["command"] = self.recording_switch
        
        self.bt_switch_ref = tk.Button(self.fr_buttons)
        self.bt_switch_ref["text"] = "Apply ref"
        self.bt_switch_ref["command"] = self.ref_switch
        
        self.bt_switch_anim = tk.Button(self)
        self.bt_switch_anim["text"] = "Pause"
        self.bt_switch_anim["command"] = self.pause_switch

        self.bt_quit = tk.Button(self, text="QUIT", fg="red", command=self.clean_quit)
        
        # create ref sliders
        self.sc_ref_x = tk.Scale(self.fr_buttons, from_=-2, to=2, resolution=0.01, length=200, label='X', orient=tk.HORIZONTAL, command=self.update_ref_queue)
        self.sc_ref_y = tk.Scale(self.fr_buttons, from_=-2, to=2, resolution=0.01, length=200, label='Y', orient=tk.HORIZONTAL, command=self.update_ref_queue)
        self.sc_ref_z = tk.Scale(self.fr_buttons, from_=-2, to=0, resolution=0.01, length=200, label='Z', orient=tk.HORIZONTAL, command=self.update_ref_queue)
        self.sc_ref_psi = tk.Scale(self.fr_buttons, from_=-180, to=180, length=200, label='Psi', orient=tk.HORIZONTAL, command=self.update_ref_queue)
        
        # create gains sliders
        self.sc_gain_kpxy = tk.Scale(self.fr_buttons, from_=0, to=2, resolution=0.01, length=200, label='Kpxy', orient=tk.HORIZONTAL, command=self.update_gain_queue)
        self.sc_gain_kpz = tk.Scale(self.fr_buttons, from_=0, to=2, resolution=0.01, length=200, label='Kpz', orient=tk.HORIZONTAL, command=self.update_gain_queue)
        self.sc_gain_kdxy = tk.Scale(self.fr_buttons, from_=0, to=2, resolution=0.01, length=200, label='Kdxy', orient=tk.HORIZONTAL, command=self.update_gain_queue)
        self.sc_gain_kdz = tk.Scale(self.fr_buttons, from_=0, to=2, resolution=0.01, length=200, label='Kdz', orient=tk.HORIZONTAL, command=self.update_gain_queue)
        
        #create plot canvas
        self.canvas = FigureCanvasTkAgg(self.f, self)
        self.canvas.show()
        self.toolbar = NavigationToolbar2TkAgg(self.canvas, self)
        self.toolbar.update()
        
        # place widgets
        self.fr_buttons.pack()
        
        self.bt_switch_motors.grid(row=0, column=0)
        self.bt_switch_control.grid(row=0, column=1)
        self.bt_switch_recording.grid(row=0, column=2)
        self.bt_switch_ref.grid(row=0, column=3)
        
        self.sc_gain_kpxy.grid(row=1, column=0, columnspan=2)
        self.sc_gain_kpz.grid(row=2, column=0, columnspan=2)
        self.sc_gain_kdxy.grid(row=3, column=0, columnspan=2)
        self.sc_gain_kdz.grid(row=4, column=0, columnspan=2)
        
        self.sc_ref_x.grid(row=1, column=2, columnspan=2)
        self.sc_ref_y.grid(row=2, column=2, columnspan=2)
        self.sc_ref_z.grid(row=3, column=2, columnspan=2)
        self.sc_ref_psi.grid(row=4, column=2, columnspan=2)
        
        self.canvas.get_tk_widget().pack(side=tk.BOTTOM,fill=tk.BOTH, expand=1)
        self.canvas._tkcanvas.pack(side=tk.BOTTOM,fill=tk.BOTH, expand=1)
        
        self.bt_switch_anim.pack()
        self.bt_quit.pack(side=tk.BOTTOM)
    
    def motors_switch(self):
        self.q_controls.put(GuiControls.ON_OFF)
    
    def control_switch(self):
        self.q_controls.put(GuiControls.CONTROL)
        if self.bt_switch_control['relief'] == tk.RAISED:
            self.bt_switch_control['relief'] = tk.SUNKEN
        else:
            self.bt_switch_control['relief'] = tk.RAISED
        
    def recording_switch(self):
        self.q_controls.put(GuiControls.RECORD)
        if self.bt_switch_recording['bg']=="red":
            self.bt_switch_recording['bg']="grey"
        else:
            self.bt_switch_recording['bg']="red"
            
    def ref_switch(self):
        self.ref_applied = not self.ref_applied
        self.update_ref_queue(None)
        if self.bt_switch_ref['relief'] == tk.RAISED:
            self.bt_switch_ref['relief'] = tk.SUNKEN
        else:
            self.bt_switch_ref['relief'] = tk.RAISED
    
    def pause_switch(self):
        self.is_anim_paused = not self.is_anim_paused
    
    def clean_quit(self):
        self.q_controls.put(GuiControls.QUIT)
        self.master.destroy()
    
    def set_ref_sliders(self, values):
        x, y, z, psi = values
        self.sc_ref_x.set(x)
        self.sc_ref_y.set(y)
        self.sc_ref_z.set(z)
        self.sc_ref_psi.set(psi)
        
    def get_ref_sliders(self):
        x = self.sc_ref_x.get()
        y = self.sc_ref_y.get()
        z = self.sc_ref_z.get()
        psi = self.sc_ref_psi.get()
        return (x, y, z, psi)
        
    def set_gain_sliders(self, values):
        kpxy, kpz, kdxy, kdz = values
        self.sc_gain_kpxy.set(kpxy)
        self.sc_gain_kpz.set(kpz)
        self.sc_gain_kdxy.set(kdxy)
        self.sc_gain_kdz.set(kdz)
        
    def get_gain_sliders(self):
        kpxy = self.sc_gain_kpxy.get()
        kpz = self.sc_gain_kpz.get()
        kdxy = self.sc_gain_kdxy.get()
        kdz = self.sc_gain_kdz.get()
        return (kpxy, kpz, kdxy, kdz)
    
    def update_gain_queue(self, _):
        try:
            self.q_gains.get_nowait()
        except:
            pass
        self.q_gains.put(self.get_gain_sliders())
        
    def update_ref_queue(self, _):
        if self.ref_applied:
            try:
                self.q_ref.get_nowait()
            except:
                pass
            self.q_ref.put(self.get_ref_sliders())
    
    def _continuous_update_plot(self):
        while True:
            try:
                d = self.q_data.get_nowait()
                # limit plotted data to 5000 samples
                if self.data.shape[0] > 5000:
                    self.data = np.roll(self.data,-1,axis=0)
                    self.data[-1,:] = d
                else:
                    self.data = np.append(self.data, d, axis=0)
            except:
                break
        if self.is_anim_paused:
            pass
        else:
            self.a1.clear()
            self.a2.clear()
            self.a3.clear()
            self.a1.plot(self.data[:,2], self.data[:,3])
            self.a1.plot(self.data[:,2], self.data[:,10])
            self.a1.plot(self.data[:,2], self.data[:,14])
            self.a2.plot(self.data[:,2], self.data[:,4])
            self.a2.plot(self.data[:,2], self.data[:,11])
            self.a2.plot(self.data[:,2], self.data[:,15])
            self.a3.plot(self.data[:,2], self.data[:,5])
            self.a3.plot(self.data[:,2], self.data[:,12])
            self.a3.plot(self.data[:,2], self.data[:,16])
            self.canvas.draw()
        self.after(1000,self._continuous_update_plot)
        