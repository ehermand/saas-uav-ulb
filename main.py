"""
Main file

@author: elie
"""

import numpy as np
import time

import scipy.io as spio

import logging
import natnetclient as natnet
import kalman_filter as kalman
import uav_control as control
import quaternion as quat
import ardrone_wrapper

import tkinter as tk
import gui
from gui import GuiControls

import erg

from multiprocessing import Process,Queue

# Set reference values
ref = np.matrix([[0],[0],[-1.2],[0]])

# Master loop
def master_loop(q_controls,q_gains,q_ref,q_data):
    global ref

    # Initialize natnet client
    client = natnet.NatClient(client_ip='127.0.0.1',data_port=1511,comm_port=1510)
    uav_tracker = client.rigid_bodies['UAV']

    # Initialize PCtoRC
    #uav_controller = pctorc.UAVController()

    #Initialize ARDrone
    uav_controller = ardrone_wrapper.ARDroneController()
    uav_controller.navdata_ready.wait()
    uav_controller.trim()
    
    # Initialize ERG
    dic = spio.loadmat("./erg/ss_v1_mat.mat")
    ss = (np.matrix(dic["A"]),np.matrix(dic["B"]),np.matrix(dic["C"]),np.matrix(dic["D"]))
    P = np.matrix(np.load("./erg/ss_v1_P.npy"))
    
    # Create constraints
    delta = 0.2
    zeta = 0.5
    c1 = erg.WallConstraint(np.array([[1,1,0]]).T,1,delta,zeta)
    c2 = erg.WallConstraint(np.array([[-1,-1,0]]).T,1,delta,zeta)
    c3 = erg.WallConstraint(np.array([[1,-1,0]]).T,1,delta,zeta)
    c4 = erg.WallConstraint(np.array([[-1,1,0]]).T,1,delta,zeta)
    #c5 = erg.SphereConstraint(np.array([[0.5,-0.5,-1.2]]).T,0.25,delta,zeta)
	
    uav_erg = erg.ERG(ss,P,[c1,c2,c3,c4],0.05,0.01)
    
    ref_mod = np.copy(ref)
    
    # Initialize variables
    is_watching = True
    is_recording = False

    measurements = np.zeros([0,18])
    
    #Main loop
    iframe = client.iFrame
    timestamp = client.timestamp
    
    while is_watching:
        # Get natnet data
        client.get_data()
    
        # Get GUI controls
        while True:
            try:
                c = q_controls.get_nowait()
                if c == GuiControls.ON_OFF:
                    if uav_controller.state.fly_mask:
                        uav_controller.landing = True
                    else:
                        uav_controller.takingoff = True
                elif c == GuiControls.EMERGENCY:
                    pass
                elif c == GuiControls.CONTROL:
                    uav_controller.controlling = not uav_controller.controlling
                elif c == GuiControls.RECORD:
                    is_recording = not is_recording
                    if not is_recording:
                        np.save(time.strftime("recordings/%Y-%m-%d-%H%M.npy",time.localtime()), measurements)
                        measurements = np.zeros([0,18])
                elif c == GuiControls.QUIT:
                    uav_controller.controlling = False
                    is_watching = False
                    time.sleep(1)
            except:
                break

        # Update gains and reference
        try:
            kpxy, kpz, kdxy, kdz = q_gains.get_nowait()
            control.Kp_pos = np.array([[kpxy, kpxy, kpz]]).T
            control.Kd_pos = np.array([[kdxy, kdxy, kdz]]).T
        except:
            pass
        try:
            r = q_ref.get_nowait()
            ref = np.matrix(r).T
        except:
            pass
            
        
        iframe_old = iframe
        iframe = client.iFrame
        """if iframe == iframe_old:
            continue"""
        
        # Update time step
        timestamp_old = timestamp
        timestamp = client.timestamp
        pc_time = time.time()
        dt = timestamp - timestamp_old
    
        # Get measurements
        p_meas = np.matrix(uav_tracker.position).T
        q_meas = np.matrix(uav_tracker.quaternion).T
        r_meas = np.matrix(quat.q_to_euler(uav_tracker.quaternion,'xyzw')).T
        
        # Filter
        p_est, v_est = kalman.update(p_meas,dt)
        
        # ERG
        x = np.concatenate((p_est,v_est),axis=0)
        ref_mod[0:3] = uav_erg.compute_reference(x,ref[0:3],ref_mod[0:3],dt)
        ref_mod[3] = ref[3] # forward psi ref
        
        # Control
        Tc,phic,thetac,psic = control.outerloop(p_est,v_est,r_meas[0],ref_mod)
        
        # Send command to UAV
        uav_controller.command = uav_controller.normalize_command(np.array([phic,thetac,Tc,psic]))
        uav_controller.send_control()
        
        # Update recordings
        meas = np.concatenate([[[iframe, pc_time, timestamp]], p_meas.T, q_meas.T, ref.T, ref_mod.T], axis=1)
        
        q_data.put(meas)
        
        if is_recording:
            measurements = np.append(measurements, meas, axis=0)
            
        time.sleep(0.002)
        
    
    # Finalize recording
    if is_recording:
        np.save(time.strftime("recordings/%Y-%m-%d-%H%M.npy",time.localtime()), measurements)
    
    uav_controller._close()
    

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    
    # Initialize queues
    q_controls = Queue()
    q_gains = Queue()
    q_ref = Queue()
    q_data = Queue()
    
    # Initialize GUI
    root = tk.Tk()
    app = gui.Application(q_controls,q_gains,q_ref,q_data,master=root)
    app.set_ref_sliders(np.array(ref.T)[0,:])
    app.set_gain_sliders((control.Kp_pos[0,0],control.Kp_pos[2,0],control.Kd_pos[0,0],control.Kd_pos[2,0]))
    
    # Create main process
    p = Process(target=master_loop, args=(q_controls,q_gains,q_ref,q_data))
    
    # Start process and GUI
    p.start()
    app.mainloop()
    
    p.join()