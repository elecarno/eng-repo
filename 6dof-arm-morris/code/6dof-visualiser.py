import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# FUNCTIONS ----------------------------------------------------------------------------------------
I = np.eye(3)

def skew(x):
    skew_x = np.array([
        [ 0,    -x[2],  x[1]],
        [ x[2],  0,    -x[0]],
        [-x[1],  x[0],  0   ]
    ])
    return skew_x

def rot(w, theta):
    """
    3x3 rotation matrix
    """
    R = I + np.sin(theta)*skew(w) + (1-np.cos(theta))*(skew(w) @ skew(w))
    return R

def pos(w, v, theta):
    """
    column vector
    """
    p = (I*theta + (1-np.cos(theta))*skew(w) + (theta - np.sin(theta))*(skew(w) @ skew(w))) @ v
    return p

def screw_to_transformation(w, v, theta):
    """
    SE(3) transformation matrix for e^{[S]theta}
    """

    T = np.eye(4)

    # case that w is a zero vector
    if np.allclose(w, 0):
        T[:3, :3] = I
        T[:3, 3] = v * theta
        return T
    
    # otherwise:
    R = rot(w, theta)
    p = pos(w, v, theta)

    T[:3, :3] = R
    T[:3, 3] = p
    return T

def poe_forward_fk(screws, thetas, M):
    T = np.eye(4)

    for (w, v), theta in zip(screws, thetas):
        T_i = screw_to_transformation(w, v, theta)
        T = T @ T_i

    T_final = T @ M
    return T_final

def plot_frame(ax, T, scale=1.0, label_prefix=""):
    origin = T[:3, 3]
    R = T[:3, :3]
    
    # x-axis
    ax.quiver(origin[0], origin[1], origin[2], R[0,0], R[1,0], R[2,0], color='r', length=scale, normalize=True)
    # y-axis
    ax.quiver(origin[0], origin[1], origin[2], R[0,1], R[1,1], R[2,1], color='g', length=scale, normalize=True)
    # z-axis
    ax.quiver(origin[0], origin[1], origin[2], R[0,2], R[1,2], R[2,2], color='b', length=scale, normalize=True)
    
    # label
    ax.text(origin[0] + 0.1, origin[1] + 0.1, origin[2] + 0.1, label_prefix, fontsize=10, weight='bold')


# TEST ---------------------------------------------------------------------------------------------
if __name__ == "__main__":
    # link lengths
    L1 = 2
    L2 = 3

    # joints
    S1 = ( np.array([0, 0, 1]), np.array([0, 0, 0]) )
    S2 = ( np.array([1, 0, 0]), np.array([0, -L1, 0]) )
    screws = [S1, S2]

    # joint positions
    thetas = [
        np.radians(0),
        np.radians(0)
    ]

    # home configuration
    M = np.array([
        [1, 0, 0, L2],
        [0, 1, 0, 0 ],
        [0, 0, 1, L1],
        [0, 0, 0, 1 ]
    ])

    # forward ik
    T_fk = poe_forward_fk(screws, thetas, M)


    # MATPLOTLIB -----------------------------------------------------------------------------------
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # space and body (end-effector) frames
    T_origin = np.eye(4)
    plot_frame(ax, T_origin, scale=1.0, label_prefix="{s}")
    plot_frame(ax, T_fk, scale=1.5, label_prefix="{b}")

    # formatting
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Robot Forward Kinematics')
    
    # scale
    ax.set_xlim(-6, 6)
    ax.set_ylim(-6, 6)
    ax.set_zlim(0, 6)

    ax.legend(loc='upper left')
    plt.show()