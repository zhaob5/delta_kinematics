# # -*- coding: utf-8 -*-
"""
SHER-3.0 Delta Inverse Kinematics Visualization

Created on Tue May 27 17:11:23 2025

@author: Botao Zhao
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

def calculate_actuator_lengths(x, y, z):
    # Platform joint positions in world frame
    p1 = np.array([x, y, z]) + rp1
    p2 = np.array([x, y, z]) + rp2
    p3 = np.array([x, y, z]) + rp3

    # Vectors from base to platform joints
    l1 = p1 - rb1
    l2 = p2 - rb2
    l3 = p3 - rb3

    # z_i = z - sqrt(l^2 - dx^2 - dy^2)
    def z_offset(vec):
        dx, dy = vec[0], vec[1]
        d_squared = dx**2 + dy**2
        if d_squared > l**2:
            return np.nan
        return z - np.sqrt(l**2 - d_squared)

    z1 = z_offset(l1)
    z2 = z_offset(l2)
    z3 = z_offset(l3)

    return z1, z2, z3

def update(val):
    x = slider_x.val
    y = slider_y.val
    z = slider_z.val

    z1, z2, z3 = calculate_actuator_lengths(x, y, z)

    ax.cla()
    
    # Replot everything
    p1 = np.array([x, y, z]) + rp1
    p2 = np.array([x, y, z]) + rp2
    p3 = np.array([x, y, z]) + rp3

    # l1 = [p1[0] - rb1[0], p1[1] - rb1[1], z1]
    # l2 = [p2[0] - rb2[0], p2[1] - rb2[1], z2]
    # l3 = [p3[0] - rb3[0], p3[1] - rb3[1], z3]

    # links
    ax.plot([rb1[0], p1[0]], [rb1[1], p1[1]], [rb1[2] + z1, p1[2]], 'g')
    ax.plot([rb2[0], p2[0]], [rb2[1], p2[1]], [rb2[2] + z2, p2[2]], 'g')
    ax.plot([rb3[0], p3[0]], [rb3[1], p3[1]], [rb3[2] + z3, p3[2]], 'g')

    ax.quiver(rb1[0], rb1[1], rb1[2], 0, 0, z1, color='b')
    ax.quiver(rb2[0], rb2[1], rb2[2], 0, 0, z2, color='b')
    ax.quiver(rb3[0], rb3[1], rb3[2], 0, 0, z3, color='b')

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
    ax.set_title(f"L1 = {z1:.2f} mm, L2 = {z2:.2f} mm, L3 = {z3:.2f} mm")

    fig.canvas.draw_idle()

# Initial pose
x0, y0, z0 = 0, 0, 225

# Set up figure with space for sliders
fig = plt.figure(figsize=(10, 8))
gs = gridspec.GridSpec(2, 1, height_ratios=[6, 1])
ax = fig.add_subplot(gs[0], projection='3d')

slider_ax_x = plt.axes([0.25, 0.15, 0.65, 0.03])
slider_ax_y = plt.axes([0.25, 0.10, 0.65, 0.03])
slider_ax_z = plt.axes([0.25, 0.05, 0.65, 0.03])

slider_x = Slider(slider_ax_x, 'X', -25, 25, valinit=x0)
slider_y = Slider(slider_ax_y, 'Y', -25, 25, valinit=y0)
slider_z = Slider(slider_ax_z, 'Z', 150, 300, valinit=z0)

# Hook update
slider_x.on_changed(update)
slider_y.on_changed(update)
slider_z.on_changed(update)

# Initial draw
update(None)
plt.tight_layout()
plt.show()
