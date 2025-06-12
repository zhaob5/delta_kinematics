# -*- coding: utf-8 -*-
"""
Created on Wed Jun 11 21:13:48 2025

@author: Botao
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import matplotlib.gridspec as gridspec

# Robot parameters
rp = 34.4773 # mm platform radius
rb = 60.6927 # mm base radius
l = 68  # mm rod length

# Base joint angles
theta1 = np.pi / 3
theta2 = np.pi
theta3 = 5 * np.pi / 3

# Base joint positions (fixed)
rb1 = np.array([np.cos(theta1)*rb, np.sin(theta1)*rb, 0])
rb2 = np.array([np.cos(theta2)*rb, np.sin(theta2)*rb, 0])
rb3 = np.array([np.cos(theta3)*rb, np.sin(theta3)*rb, 0])

# Platform offsets (relative to center)
rp1 = np.array([np.cos(theta1)*rp, np.sin(theta1)*rp, 0])
rp2 = np.array([np.cos(theta2)*rp, np.sin(theta2)*rp, 0])
rp3 = np.array([np.cos(theta3)*rp, np.sin(theta3)*rp, 0])

def update(val):
    q1 = slider_q1.val
    q2 = slider_q2.val
    q3 = slider_q3.val
    
    L1 = np.array([0,0,q1])
    L2 = np.array([0,0,q2])
    L3 = np.array([0,0,q3])
    
    e1 = L1 + rb1 - rp1
    e2 = L2 + rb2 - rp2
    e3 = L3 + rb3 - rp3
    
    h1 = ((e2[0]**2 + e2[1]**2 + e2[2]**2) - (e1[0]**2 + e1[1]**2 + e1[2]**2))/2
    h2 = ((e3[0]**2 + e3[1]**2 + e3[2]**2) - (e1[0]**2 + e1[1]**2 + e1[2]**2))/2

    x21 = (e2 - e1)[0]
    y21 = (e2 - e1)[1]
    z21 = (e2 - e1)[2]
    
    x31 = (e3 - e1)[0]
    y31 = (e3 - e1)[1]
    z31 = (e3 - e1)[2]    
    
    k1 = (y31*z21/y21 - z31)/(x31 - y31*x21/y21)
    k2 = (h2 - y31*h1/y21)/(x31 - y31*x21/y21)
    k3 = (x31*z21/x21 - z31)/(y31 - x31*y21/x21)
    k4 = (h2 - x31*h1/x21)/(y31 - x31*y21/x21)
    
    T1 = k1**2 + k3**2 + 1
    T2 = k1*k2 + k3*k4 - e1[0]*k1 - e1[1]*k3 - e1[2]
    T3 = k2**2 + k4**2 -2*e1[0]*k2 - 2*e1[1]*k4 - l**2 + e1[0]**2 + e1[1]**2 + e1[2]**2
    
    z = (-T2 + np.sqrt(T2**2 - T1*T3))/T1
    x = k1*z + k2
    y = k3*z + k4
    
    l1 = np.array([x, y, z]) - e1
    
    ax.cla()
    
    # Replot everything
    p1 = np.array([x, y, z]) + rp1
    p2 = np.array([x, y, z]) + rp2
    p3 = np.array([x, y, z]) + rp3

    # l1 = [p1[0] - rb1[0], p1[1] - rb1[1], z1]
    # l2 = [p2[0] - rb2[0], p2[1] - rb2[1], z2]
    # l3 = [p3[0] - rb3[0], p3[1] - rb3[1], z3]

    # links
    ax.plot([rb1[0], p1[0]], [rb1[1], p1[1]], [rb1[2] + q1, p1[2]], 'g')
    ax.plot([rb2[0], p2[0]], [rb2[1], p2[1]], [rb2[2] + q2, p2[2]], 'g')
    ax.plot([rb3[0], p3[0]], [rb3[1], p3[1]], [rb3[2] + q3, p3[2]], 'g')

    ax.quiver(rb1[0], rb1[1], rb1[2], 0, 0, q1, color='b')
    ax.quiver(rb2[0], rb2[1], rb2[2], 0, 0, q2, color='b')
    ax.quiver(rb3[0], rb3[1], rb3[2], 0, 0, q3, color='b')

    ax.quiver(0, 0, 0, x, y, z, color='k')

    # Platform vectors
    ax.quiver(x, y, z, rp1[0], rp1[1], rp1[2], color='r')
    ax.quiver(x, y, z, rp2[0], rp2[1], rp2[2], color='r')
    ax.quiver(x, y, z, rp3[0], rp3[1], rp3[2], color='r')

    ax.set_xlim([-200, 200])
    ax.set_ylim([-200, 200])
    ax.set_zlim([0, 400])
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"x = {x:.2f} mm, y = {y:.2f} mm, z = {z:.2f} mm \n l = {np.linalg.norm(l1):.2f} mm")

    fig.canvas.draw_idle()

# Initial pose
q10, q20, q30 = 50, 50, 50

# Set up figure with space for sliders
fig = plt.figure(figsize=(10, 8))
gs = gridspec.GridSpec(2, 1, height_ratios=[6, 1])
ax = fig.add_subplot(gs[0], projection='3d')

slider_ax_q1 = plt.axes([0.25, 0.15, 0.65, 0.03])
slider_ax_q2 = plt.axes([0.25, 0.10, 0.65, 0.03])
slider_ax_q3 = plt.axes([0.25, 0.05, 0.65, 0.03])

slider_q1 = Slider(slider_ax_q1, 'q1', 0, 100, valinit=q10)
slider_q2 = Slider(slider_ax_q2, 'q2', 0, 100, valinit=q20)
slider_q3 = Slider(slider_ax_q3, 'q3', 0, 100, valinit=q30)

# Hook update
slider_q1.on_changed(update)
slider_q2.on_changed(update)
slider_q3.on_changed(update)

# Initial draw
update(None)
plt.tight_layout()
plt.show()