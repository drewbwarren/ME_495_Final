#!/usr/bin/env python

import numpy as np

def axis_angle_to_quat(w, th):
    return np.array(np.hstack((np.cos(th/2.0), np.array(w)*np.sin(th/2.0))))

def so3_to_axis_angle(R):
    th = np.arccos((R.trace() - 1)/2)
    if (th <= 1e-12):
        th = 0.0
        w = 3*[1/np.sqrt(3)]
    else:
        w = 1/(2*np.sin(th))*np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
        if any(map(np.isinf, w)) or any(map(np.isnan, w)):
            th = 0.0
            w = 3*[1/np.sqrt(3)]
    return w, th

def so3_to_quat(R):
    w,th = so3_to_axis_angle(R)
    return axis_angle_to_quat(w, th)


