import numpy as np

# --- vector and matrix handling -------------------------------------------------------------------
def unit(v: np.ndarray) -> np.ndarray:
    if v.shape != (3,):
        raise ValueError("Input vector must have exactly 3 elements.")
    
    mag = np.linalg.norm(v)
    if mag == 0:
        return np.zeros(3)

    return v / mag

def skew(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v).ravel()
    if v.shape != (3,):
        raise ValueError("Input vector must have exactly 3 elements.")
        
    return np.array([
        [ 0,    -v[2],  v[1]],
        [ v[2],  0,    -v[0]],
        [-v[1],  v[0],  0   ]
    ])

def skew2(v: np.ndarray) -> np.ndarray:
    # FIXED: Matrix square of skew(v) guarantees correct signs
    S = skew(v)
    return S @ S


# --- rotation matrix via rodrigues formula --------------------------------------------------------
def rot(w: np.ndarray, theta: float) -> np.ndarray:
    if w.shape != (3,):
        raise ValueError("Input axis must have exactly 3 elements.")
    
    w_unit = unit(w)
    return np.identity(3) + np.sin(theta)*skew(w_unit) + (1 - np.cos(theta))*skew2(w_unit)


# --- screw, twist, and wrench ---------------------------------------------------------------------
def screw(w: np.ndarray, v: np.ndarray) -> np.ndarray:
    w_unit = unit(w)
    return np.array([w_unit[0], w_unit[1], w_unit[2], v[0], v[1], v[2]])

def twist(w: np.ndarray, v: np.ndarray) -> np.ndarray:
    return screw(w, v)

def wrench(m: np.ndarray, f: np.ndarray) -> np.ndarray:
    return np.array([m[0], m[1], m[2], f[0], f[1], f[2]])


# --- rigid-body motion ----------------------------------------------------------------------------
def trans(S: np.ndarray, theta: float) -> np.ndarray:
    w_raw = np.array([S[0], S[1], S[2]])
    v = np.array([S[3], S[4], S[5]])

    I = np.identity(3)
    w_norm = np.linalg.norm(w_raw)

    if np.isclose(w_norm, 0):
        p = (v * theta).reshape(3, 1)
        return np.block([
            [I,                p],
            [np.zeros((1, 3)), 1]
        ])
    else:
        w_unit = w_raw / w_norm
        R = rot(w_unit, theta)
        G = I * theta + (1 - np.cos(theta)) * skew(w_unit) + (theta - np.sin(theta)) * skew2(w_unit)
        p = (G @ v).reshape(3, 1)
        
        return np.block([
            [R,                p],
            [np.zeros((1, 3)), 1]
        ])
    
def ad_rep(T: np.ndarray) -> np.ndarray:
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    pR = skew(p) @ R

    return np.block([
        [R,   np.zeros((3, 3))],
        [pR,  R               ]
    ])


# --- forward kinematics ---------------------------------------------------------------------------
def PoE_space(screws: list, thetas: np.ndarray, M: np.ndarray) -> np.ndarray:
    T = np.identity(4)

    for i in range(len(screws)):
        T = T @ trans(screws[i], thetas[i])
    
    return T @ M

def PoE_body(screws: list, thetas: np.ndarray, M: np.ndarray) -> np.ndarray:
    M_inv = np.linalg.inv(M)
    Ad_M_inv = ad_rep(M_inv)
    
    T = M.copy()
    for i in range(len(screws)):
        body_screw = Ad_M_inv @ screws[i]
        T = T @ trans(body_screw, thetas[i])

    return T


# --- jacobians ------------------------------------------------------------------------------------
def jac_column(screws: list, thetas: np.ndarray, col: int) -> np.ndarray:
    T = np.identity(4)
    for i in range(col):
        T = T @ trans(screws[i], thetas[i])

    return ad_rep(T) @ screws[col]


def jac_space(screws: list, thetas: np.ndarray) -> np.ndarray:
    columns = [jac_column(screws, thetas, i) for i in range(len(screws))]
    return np.column_stack(columns)


def jac_body(screws: list, thetas: np.ndarray, M: np.ndarray) -> np.ndarray:
    T_fk = PoE_space(screws, thetas, M)
    Js = jac_space(screws, thetas)
    return ad_rep(np.linalg.inv(T_fk)) @ Js


# --- MAIN -----------------------------------------------------------------------------------------
if __name__ == "__main__":
    L1, L2 = 0.3, 0.3

    thetas = np.array([
        np.radians(-90),
        np.radians(0), 
        np.radians(0),
        np.radians(0),
        np.radians(0), 
        np.radians(0), 
        np.radians(0)
    ])

    screws = [
        screw(np.array([ 1,  0,  0 ]), np.array([ 0,  0,         0])),  # S1: q = [0,0,0]
        screw(np.array([ 0, -1,  0 ]), np.array([ 0,  0,         0])),  # S2: q = [0,0,0]
        screw(np.array([ 0,  0, -1 ]), np.array([ 0,  0,         0])),  # S3: q = [0,0,0]

        screw(np.array([ 1,  0,  0 ]), np.array([ 0, -L1,        0])),  # S4: q = [0,0,-L1]

        screw(np.array([ 0,  0, -1 ]), np.array([ 0, -(L1 + L2), 0])),  # S5: q = [0,0,-(L1+L2)] -> v = 0
        screw(np.array([ 1,  0,  0 ]), np.array([ 0, -(L1 + L2), 0])),  # S6: q = [0,0,-(L1+L2)] -> v = [0, -(L1+L2), 0]
        screw(np.array([ 0,  0, -1 ]), np.array([ 0, -(L1 + L2), 0]))   # S7: q = [0,0,-(L1+L2)] -> v = 0
    ]

    M = np.array([
        [ 1,  0,  0,  0       ],
        [ 0, -1,  0,  0       ],
        [ 0,  0, -1, -(L1+L2) ],
        [ 0,  0,  0,  1       ]
    ])

    T = PoE_space(screws, thetas, M)
    Js = jac_space(screws, thetas)

    Fs = wrench(
        np.array([-5.886, 0, 0]),
        np.array([0, 0, 0])
    )

    tau = np.transpose(Js) @ Fs

    # printing -----------------------------------------------------------------
    x_b = np.array([ T[0, 0], T[1, 0], T[2, 0] ])
    y_b = np.array([ T[0, 1], T[1, 1], T[2, 1] ])
    z_b = np.array([ T[0, 2], T[1, 2], T[2, 2] ])
    p_b = np.array([ T[0, 3], T[1, 3], T[2, 3] ])

    print("Robot Configuration:")
    print(np.round(thetas, 3))
    print("Joint Screw Axes:")
    print(np.round(screws, 3))

    print("\nPoE Transformation Matrix:")
    print(np.round(T, 3))
    print("Jacobian Matrix:")
    print(np.round(Js, 3))

    print("\nEnd-Effector Orientation:")
    print(f"\tx_b: {np.round(x_b, 3)}")
    print(f"\ty_b: {np.round(y_b, 3)}")
    print(f"\tz_b: {np.round(z_b, 3)}")
    print("End-Effector Position:")
    print(f"\tp_b: {np.round(p_b, 3)}")

    print("\nApplied Wrench:")
    print(f"\tF_s: {np.round(Fs, 3)}")
    print("Joint Torques (Nm):")
    print(f"\ttau: {np.round(tau, 3)}")
