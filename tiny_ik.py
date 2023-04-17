import tinyik
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import open3d as o3d

pi = np.pi
c = np.cos
s = np.sin

arm = tinyik.Actuator(
    [[.0, .55, 0], 'z', [2.43, .0, .0], 'y', [1.64, .0, .0], 'y', [.0, .0, .4], 'y', [.0, -.63, .25], 'z'])

# Define the joint angles (in radians)
q = np.array([0.0, 0.0, pi / 2, 0.0, pi / 2])

# arm.angles = np.deg2rad(q)
# arm.angles = q

# tinyik.visualize(arm)

# Define the DH parameters of the manipulator (a, alpha, d, theta)
dh_params = np.array([
    [5.5, -pi / 2, -11, 0.0],
    [24.3, 0.0, 0.0, 0.0],
    [16.4, 0.0, 0.0, 0.0],
    [0.0, pi / 2, 4.0, 0.0],
    [6.3, -pi / 2, 2.5, -pi / 2]])

# Define the end-effector pose
T_desired = np.array([
    [1.0, 0.0, 0.0, 15],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]
])


# Define a function to compute the homogeneous transformation matrix
# from DH parameters for a given joint angle
def compute_transformation(alpha, a, d, theta):
    return np.array([
        [c(theta), -s(theta) * c(alpha), s(theta) * s(alpha), a * c(theta)],
        [s(theta), c(theta) * c(alpha), -c(theta) * s(alpha), a * s(theta)],
        [0, s(alpha), c(alpha), d],
        [0, 0, 0, 1.0]])


# compute forward kinematics
T_matrices = []
for i in range(dh_params.shape[0]):
    a, alpha, d, theta = dh_params[i]
    T_i = np.array([
        [c(theta), -s(theta) * c(alpha), s(theta) * s(alpha), a * c(theta)],
        [s(theta), c(theta) * c(alpha), -c(theta) * s(alpha), a * s(theta)],
        [0, s(alpha), c(alpha), d],
        [0, 0, 0, 1]
    ])
    T_matrices.append(T_i)

T = np.eye(4)
for i in range(len(T_matrices)):
    T = T @ T_matrices[i]
end_effector_pos = T[:3, 3]
print(f"end_effector_pose: {end_effector_pos}")
end_effector_pos = [25, 2.3, 0.0]
print(f"end_effector_pose: {end_effector_pos}")

# compute joint angles using inverse kinematics
q1 = np.arctan2(end_effector_pos[1], end_effector_pos[0])
q3 = np.sqrt(end_effector_pos[0] ** 2 + end_effector_pos[1] ** 2 + (end_effector_pos[2] - dh_params[2, 0]) ** 2) - (dh_params[3, 0] - dh_params[4, 0])
q2 = -np.pi / 2
q4 = np.arctan2(T[1, 0] * np.cos(q1) + T[0, 0] * np.sin(q1), -T[1, 1] * np.cos(q1) - T[0, 1] * np.sin(q1))
q5 = np.arctan2(-T[2, 1] * np.cos(q1) * np.cos(q4) - T[2, 0] * np.sin(q1) * np.cos(q4) + T[2, 2] * np.sin(q4),
                T[2, 1] * np.sin(q1) - T[2, 0] * np.cos(q1))

q_dot = [q1, q2, q3, q4, q5]

# print("Joint angles (in radians):", q_dot)

# Update the joint angles
q_new = q + q_dot
# print(f"New joint angles: {np.rad2deg(q_new)}")

# Plot the manipulator
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

colors = ['r', 'g', 'b', 'y', 'm']
prev_pos = np.zeros((3,))
for i in range(5):
    T_i = np.eye(4)
    # print(f"T_i eye: {T_i}")
    for j in range(i):
        T_i = T_i @ T_matrices[j]
    # print(f"T_i final: {T_i}")
    pos = T_i[:3, 3]
    ax.plot([prev_pos[0], pos[0]], [prev_pos[1], pos[1]], [prev_pos[2], pos[2]], colors[i])
    prev_pos = pos
ax.plot([prev_pos[0], end_effector_pos[0]], [prev_pos[1], end_effector_pos[1]], [prev_pos[2], end_effector_pos[2]], 'k')

# Set the axis limits and labels
ax.set_xlim(-55.0, 55.0)
ax.set_ylim(-55.0, 55.0)
ax.set_zlim(-55.0, 55.0)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()
