# --- IMPORTS --------------------------------------------------------------------------------------
import ctypes
import threading
import time
import tkinter as tk
from tkinter import ttk
import mujoco
import mujoco.viewer
import numpy as np

# --- ENABLE HIGH DPI AWARENESS FOR WINDOWS 2.5K DISPLAY -------------------------------------------
try:
    # Set Per-Monitor DPI Awareness (V2) for Windows 10/11
    ctypes.windll.shcore.SetProcessDpiAwareness(2)
except Exception:
    try:
        # Fallback to System DPI Awareness
        ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass

# --- LOAD MODEL AS GLOBAL -------------------------------------------------------------------------
model = mujoco.MjModel.from_xml_path("../6dof-gen_mujoco/scene.xml")
data = mujoco.MjData(model)

# --- GLOBAL TARGET POSE STATE ---------------------------------------------------------------------
target_pose = {
    "x": 0.0,
    "y": -0.1,
    "z": 0.15,
    "phi_x": np.pi / 2,
    "phi_y": 0.0,
    "phi_z": 0.0,
}


# --- PURE ALGEBRAIC IK SOLVER (REWRITTEN WRIST SOLVER) --------------------------------------------
def ik_solver(T_3d, phi_x, phi_y, phi_z):
    # Link lengths matching model geometry
    L1 = 0.027  # Base height to Shoulder axis (J2)
    L2 = 0.120  # Upper arm length (J2 to J3)
    L3 = 0.060  # Elbow to Forearm Roll axis (J3 to J4)
    L4 = 0.064  # Forearm segment (J4 to J5)
    L5 = 0.025  # Wrist Pitch to Tool Flange Tip (J5 to J6 tip)

    # 1. Target End-Effector Rotation Matrix (XYZ intrinsic / ZYX extrinsic)
    cx, cy, cz = np.cos([phi_x, phi_y, phi_z])
    sx, sy, sz = np.sin([phi_x, phi_y, phi_z])

    Rot_E = np.array([
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx],
    ])

    # 2. Wrist Center Position (J5 axis center)
    # Offset along the Z-axis of the tool flange frame
    T_offset = T_3d - L5 * Rot_E[:, 2]

    # 3. Base Joint Angle (Theta 1 - Yaw around Z)
    theta1 = np.arctan2(T_offset[1], T_offset[0])

    # 4. Planar 2R IK for Shoulder (Theta 2) & Elbow (Theta 3)
    r = np.sqrt(T_offset[0] ** 2 + T_offset[1] ** 2)
    z = T_offset[2] - L1

    d_sq = r**2 + z**2
    d = np.sqrt(d_sq)

    gamma = np.arctan2(z, r)

    cos_alpha = np.clip(
        (d_sq + L2**2 - (L3 + L4) ** 2) / (2 * L2 * d), -1.0, 1.0
    )
    alpha = np.arccos(cos_alpha)

    cos_beta = np.clip(
        (L2**2 + (L3 + L4) ** 2 - d_sq) / (2 * L2 * (L3 + L4)), -1.0, 1.0
    )
    beta = np.arccos(cos_beta)

    # Elbow-Up / Shoulder-Forward configuration
    theta2 = gamma + alpha
    theta3 = beta - np.pi

    # 5. Forward Kinematics Matrix to Link 3 (Rot_03)
    c1, s1 = np.cos(theta1), np.sin(theta1)
    c23, s23 = np.cos(theta2 + theta3), np.sin(theta2 + theta3)

    R_z1 = np.array([
        [c1, -s1, 0.0],
        [s1, c1, 0.0],
        [0.0, 0.0, 1.0],
    ])

    R_y23 = np.array([
        [c23, 0.0, s23],
        [0.0, 1.0, 0.0],
        [-s23, 0.0, c23],
    ])

    Rot_03 = R_z1 @ R_y23

    # 6. Relative Spherical Wrist Rotation Matrix (Z-Y-Z Kinematics)
    R_36 = Rot_03.T @ Rot_E

    # 7. Extract Wrist Euler Angles: Theta 4 (Roll), Theta 5 (Pitch), Theta 6 (Spin)
    sy = np.sqrt(R_36[0, 2] ** 2 + R_36[1, 2] ** 2)

    if sy > 1e-6:
        theta5 = np.arctan2(sy, R_36[2, 2])
        theta4 = np.arctan2(R_36[1, 2], R_36[0, 2])
        theta6 = np.arctan2(R_36[2, 1], -R_36[2, 0])
    else:
        # Singularity handling when wrist pitch (Theta 5) is 0
        theta5 = 0.0
        theta4 = 0.0
        theta6 = np.arctan2(-R_36[0, 1], R_36[0, 0])

    return [theta1, theta2, theta3, theta4, theta5, theta6]


# --- DYNAMIC AXES HELPER --------------------------------------------------------------------------
def add_target_axes(viewer, position, R_matrix, scale=0.04, radius=0.002):
    """Draws RGB coordinate axes at the target pose in the MuJoCo viewer."""
    viewer.user_scn.ngeom = 0
    colors = [
        np.array([1.0, 0.0, 0.0, 1.0]),  # Red
        np.array([0.0, 1.0, 0.0, 1.0]),  # Green
        np.array([0.0, 0.0, 1.0, 1.0]),  # Blue
    ]

    for i in range(3):
        axis_dir = R_matrix[:, i]
        start_pos = position
        end_pos = position + axis_dir * scale

        geom_idx = viewer.user_scn.ngeom
        if geom_idx >= viewer.user_scn.maxgeom:
            break

        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[geom_idx],
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=np.array([radius, 0, 0]),
            pos=start_pos,
            mat=R_matrix.flatten(),
            rgba=colors[i],
        )

        mujoco.mjv_connector(
            viewer.user_scn.geoms[geom_idx],
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            width=radius,
            from_=start_pos,
            to=end_pos,
        )

        viewer.user_scn.ngeom += 1


# --- SIMULATION THREAD ----------------------------------------------------------------------------
def run_simulation():
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # Read thread-safe pose copy
            t_3d = np.array(
                [target_pose["x"], target_pose["y"], target_pose["z"]]
            )
            px = target_pose["phi_x"]
            py = target_pose["phi_y"]
            pz = target_pose["phi_z"]

            # Compute IK
            joint_angles = ik_solver(t_3d, px, py, pz)

            # Compute End-effector Rotation Matrix for dynamic target frame
            cx, cy, cz = np.cos([px, py, pz])
            sx, sy, sz = np.sin([px, py, pz])

            Rot_E = np.array([
                [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
                [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
                [-sy, cy * sx, cy * cx],
            ])

            # Apply actuator control
            data.ctrl[model.actuator("base").id] = -joint_angles[0] - np.pi / 2
            data.ctrl[model.actuator("shoulder").id] = -joint_angles[1] + np.pi / 2
            data.ctrl[model.actuator("elbow").id] = -joint_angles[2]
            data.ctrl[model.actuator("wrist1").id] = joint_angles[3]
            data.ctrl[model.actuator("wrist2").id] = joint_angles[4]
            data.ctrl[model.actuator("wrist3").id] = joint_angles[5]

            # Render dynamic frame axes at target location
            add_target_axes(viewer, t_3d, Rot_E)

            # Step physics forward & refresh viewer
            mujoco.mj_step(model, data)
            viewer.sync()

            # Real-time simulation timing
            time_until_next_step = model.opt.timestep - (
                time.time() - step_start
            )
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


# --- TKINTER UI CLASS (HIGH DPI OPTIMIZED) -------------------------------------------------------
class IKControlApp:

    def __init__(self, root):
        self.root = root
        self.root.title("6-DOF IK Target Control")

        # Query display scaling factor
        scaling_factor = self.root.winfo_fpixels("1i") / 72.0
        self.root.tk.call("tk", "scaling", scaling_factor)

        # Set DPI-aware window size and padding
        self.root.geometry("540x660")
        self.root.resizable(True, True)

        # Configure style font scaling
        style = ttk.Style()
        style.configure(".", font=("Segoe UI", 10))

        # Control Configuration: (key, label, min, max, initial, resolution)
        self.controls = [
            ("x", "Target X (m)", -0.3, 0.3, target_pose["x"], 0.005),
            ("y", "Target Y (m)", -0.3, 0.3, target_pose["y"], 0.005),
            ("z", "Target Z (m)", -0.1, 0.3, target_pose["z"], 0.005),
            ("phi_x", "Phi X (rad)", -np.pi, np.pi, target_pose["phi_x"], 0.05),
            ("phi_y", "Phi Y (rad)", -np.pi, np.pi, target_pose["phi_y"], 0.05),
            ("phi_z", "Phi Z (rad)", -np.pi, np.pi, target_pose["phi_z"], 0.05),
        ]

        self.vars = {}

        # Build UI layout with padding suited for high resolution
        main_frame = ttk.Frame(root, padding="15")
        main_frame.pack(fill=tk.BOTH, expand=True)

        for key, label, min_val, max_val, init_val, res in self.controls:
            frame = ttk.LabelFrame(main_frame, text=label, padding="8")
            frame.pack(fill=tk.X, pady=6)

            var = tk.DoubleVar(value=init_val)
            self.vars[key] = var

            slider = ttk.Scale(
                frame,
                from_=min_val,
                to=max_val,
                variable=var,
                orient=tk.HORIZONTAL,
                command=lambda val, k=key: self.update_target(k),
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)

            spinbox = ttk.Spinbox(
                frame,
                from_=min_val,
                to=max_val,
                increment=res,
                textvariable=var,
                width=8,
                command=lambda k=key: self.update_target(k),
            )
            spinbox.pack(side=tk.RIGHT, padx=8)
            spinbox.bind(
                "<Return>", lambda event, k=key: self.update_target(k)
            )

    def update_target(self, key):
        target_pose[key] = self.vars[key].get()


# --- MAIN ENTRY POINT -----------------------------------------------------------------------------
if __name__ == "__main__":
    # Launch MuJoCo viewer in a separate daemon thread
    sim_thread = threading.Thread(target=run_simulation, daemon=True)
    sim_thread.start()

    # Launch Tkinter UI loop in the main thread
    root = tk.Tk()
    app = IKControlApp(root)
    root.mainloop()