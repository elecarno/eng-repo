# --- IMPORTS --------------------------------------------------------------------------------------
import time
import serial
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R
import tkinter as tk
from tkinter import ttk

# --- SERIAL & MOVEMENT CONFIGURATION --------------------------------------------------------------
SERIAL_PORT = 'COM3' 
BAUD_RATE   = 9600
MOVE_TIME   = 2000  # Movement time in milliseconds

# --- VARIABLES (MUJoCo MODEL FOR IK & KINEMATICS) --------------------------------------------------
model = mujoco.MjModel.from_xml_path("../morris-v2_mujoco/scene.xml")
data  = mujoco.MjData(model)

# disable contact physics
model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT

# get actuator IDs and corresponding joint addresses
actuator_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
actuator_ids = [model.actuator(name).id for name in actuator_names]
joint_ids = [model.actuator_trnid[act_id, 0] for act_id in actuator_ids]
qpos_addrs = [model.jnt_qposadr[j_id] for j_id in joint_ids]

# Extract joint limits directly from the MuJoCo model (in radians)
joint_limits = [model.jnt_range[j_id] for j_id in joint_ids]

# Find end-effector body
ee_body_name = "m_v2_end_effector"
if ee_body_name not in [model.body(i).name for i in range(model.nbody)]:
    ee_body_name = model.body(model.nbody - 1).name
ee_body_id = model.body(ee_body_name).id

def disable_collision_on_body(model, body_name):
    try:
        body1_id = model.body(body_name).id
        for geom_id in range(model.ngeom):
            if model.geom_bodyid[geom_id] == body1_id:
                model.geom_contype[geom_id] = 0
                model.geom_conaffinity[geom_id] = 0
    except KeyError:
        pass

disable_collision_on_body(model, "m_v2_base")

# Servo mapping configurations
servo_home_deg = np.array([90.0, 0.0, 0.0, 0.0, 90.0, 0.0])
servo_directions = np.array([-1.0, -1.0, -1.0, -1.0, -1.0, 1.0])

# --- TKINTER ON-DEMAND IK CONTROLLER APPLICATION --------------------------------------------------
class RobotIKApp:
    def __init__(self, root):
        self.root = root
        self.root.title("MV2 Robot - On-Demand IK Controller")
        self.root.geometry("450x620")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Initialize Serial Connection
        print(f"Connecting to real robot on {SERIAL_PORT} at {BAUD_RATE} baud...")
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # Wait for Arduino UNO reset on connection
            print("Serial connection established.")
            # Home robot on startup
            self.ser.write(b"home 2000\n")
            self.ser.flush()
            time.sleep(2.5)
        except Exception as e:
            print(f"Could not connect to serial port: {e}")
            self.ser = None

        # Target Variables (meters and degrees: X, Y, Z, Roll, Pitch, Yaw)
        self.target_x = tk.DoubleVar(value=0.05)
        self.target_y = tk.DoubleVar(value=0.05)
        self.target_z = tk.DoubleVar(value=0.35)
        self.target_roll = tk.DoubleVar(value=0.0)
        self.target_pitch = tk.DoubleVar(value=0.0)
        self.target_yaw = tk.DoubleVar(value=90.0)

        self.create_widgets()

    def create_widgets(self):
        frame = ttk.LabelFrame(self.root, text="Target End-Effector Pose", padding=15)
        frame.pack(fill="both", expand=True, padx=15, pady=15)

        # Helper function to create labeled sliders
        def add_slider(label_text, from_val, to_val, var, default_str, res_label_list):
            ttk.Label(frame, text=label_text).pack(anchor="w", pady=(4, 0))
            slider = ttk.Scale(frame, from_=from_val, to=to_val, variable=var, orient="horizontal")
            slider.pack(fill="x", pady=(0, 2))
            val_lbl = ttk.Label(frame, text=default_str)
            val_lbl.pack(anchor="e", pady=(0, 6))
            
            # Update label text dynamically when slider moves
            def on_slide(val, lbl=val_lbl, v=var):
                lbl.config(text=f"{float(val):.2f}")
            slider.config(command=on_slide)
            return slider

        # Position Sliders
        add_slider("X Position (m):", -0.2, 0.3, self.target_x, "0.05 m", None)
        add_slider("Y Position (m):", -0.3, 0.3, self.target_y, "0.05 m", None)
        add_slider("Z Position (m):",  0.05, 0.5, self.target_z, "0.35 m", None)

        # Orientation Sliders (Roll, Pitch, Yaw)
        add_slider("Roll (°):", -180, 180, self.target_roll, "0.0°", None)
        add_slider("Pitch (°):", -180, 180, self.target_pitch, "0.0°", None)
        add_slider("Yaw (°):", -180, 180, self.target_yaw, "90.0°", None)

        # Solve & Send Button
        self.solve_btn = ttk.Button(self.root, text="Solve IK & Move Robot", command=self.solve_and_move)
        self.solve_btn.pack(fill="x", padx=15, pady=5)

        # Status Label
        self.status_label = ttk.Label(self.root, text="Status: Ready", foreground="green")
        self.status_label.pack(pady=10)

    def solve_and_move(self):
        self.status_label.config(text="Status: Solving IK...", foreground="blue")
        self.root.update()

        # Read target values from sliders
        t_pos = np.array([self.target_x.get(), self.target_y.get(), self.target_z.get()])
        t_eul = np.array([self.target_roll.get(), self.target_pitch.get(), self.target_yaw.get()])
        
        # Convert Euler angles (Roll, Pitch, Yaw -> intrinsic ZYX / Tait-Bryan convention) to rotation matrix
        target_rot = R.from_euler('ZYX', t_eul, degrees=True).as_matrix()

        # Reset initial guess for stability or keep close to current
        for qpos_idx in qpos_addrs:
            data.qpos[qpos_idx] = 0.0
        mujoco.mj_forward(model, data)

        # Run Damped Least-Squares Numerical IK Solver
        max_iterations = 400
        lambda_damping = 1e-2
        dt_ik = 0.1
        converged = False

        for iteration in range(max_iterations):
            mujoco.mj_forward(model, data)
            
            curr_pos = data.xpos[ee_body_id]
            curr_mat = data.xmat[ee_body_id].reshape(3, 3)
            
            pos_err = t_pos - curr_pos
            
            # Match 6-axis orientation error (aligning full rotation matrix)
            # Using rotation vector error from rotation matrix difference
            R_err = target_rot @ curr_mat.T
            rot_err = R.from_matrix(R_err).as_rotvec()
            
            err = np.hstack((pos_err, rot_err))
            if np.linalg.norm(err) < 1e-4:
                converged = True
                break
                
            jac_pos = np.zeros((3, model.nv))
            jac_rot = np.zeros((3, model.nv))
            mujoco.mj_jacBody(model, data, jac_pos, jac_rot, ee_body_id)
            
            nv_indices = [model.jnt_dofadr[j_id] for j_id in joint_ids]
            J = np.vstack((jac_pos[:, nv_indices], jac_rot[:, nv_indices]))
            
            # Damped Least Squares inverse
            J_pinv = J.T @ np.linalg.inv(J @ J.T + (lambda_damping**2) * np.eye(6))
            dq = J_pinv @ err
            
            for i, qpos_idx in enumerate(qpos_addrs):
                data.qpos[qpos_idx] += dq[i] * dt_ik
                min_lim, max_lim = joint_limits[i]
                data.qpos[qpos_idx] = np.clip(data.qpos[qpos_idx], min_lim, max_lim)

        mujoco.mj_forward(model, data)
        solved_joints_rad = np.array([data.qpos[qpos_idx] for qpos_idx in qpos_addrs])
        solved_joints_deg = np.degrees(solved_joints_rad)

        # Map to physical servo angles
        raw_servo_deg = (solved_joints_deg * servo_directions) + servo_home_deg
        solved_servo_deg = np.clip(raw_servo_deg, 0.0, 180.0)

        print(f"\n--- IK Result (Converged: {converged}) ---")
        print(f"Solved Joint Angles (deg): {solved_joints_deg}")
        print(f"Clamped Servo Angles (Deg): {solved_servo_deg}")

        # Transmit via Serial to Real Robot
        if self.ser and self.ser.is_open:
            self.status_label.config(text="Status: Moving robot...", foreground="orange")
            self.root.update()
            
            for channel, angle in enumerate(solved_servo_deg):
                cmd = f"joint {channel} {angle:.1f} {MOVE_TIME}\n"
                self.ser.write(cmd.encode('utf-8'))
                time.sleep(0.05)
            self.ser.flush()
            
            # Wait for movement to finish
            time.sleep(MOVE_TIME / 1000.0)

        status_text = "Status: Solved & Moved!" if converged else "Status: Reached max iterations (approx)"
        self.status_label.config(text=status_text, foreground="green")

    def on_closing(self):
        self.status_label.config(text="Status: Homing & Disconnecting...", foreground="orange")
        self.root.update()
        
        if self.ser and self.ser.is_open:
            print("\nReturning robot to home position...")
            self.ser.write(b"home 2000\n")
            self.ser.flush()
            time.sleep(2.5)  # Wait for homing movement to finish
            self.ser.close()
            print("Disconnected cleanly from serial port.")
            
        self.root.destroy()

# --- MAIN ENTRY POINT -----------------------------------------------------------------------------
def main():
    root = tk.Tk()
    app = RobotIKApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()