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
    L_1x = 0.075
    L_1y = 0.050
    L_2x = 0.075
    L_2y = 0.050
    L_3z = 0.125
    L_4x = 0.025
    L_4z = 0.100
    L_5x = 0.030
    L_5z = 0.100
    L_6x = 0.060
    L_6z = 0.025
    L_7z = 0.075
    L_8z = 0.050

    m_i = [
        0.7, 0.6, 0.5, 0.5,
        0.4, 0.4, 0.1
    ]

    r_i = [
        0.132, 0.066, 0.132, 0.132,
        0.096, 0.096, 0.096
    ]

    l_i = [
        0.2, 0.2, 0.175, 0.2,
        0.05, 0.05, 0.05
    ]

    # --- INVERSE DYNAMICS DEFINITIONS -------------------------------------------------------------
    def inertia_cyl(m, r, l) -> np.ndarray:
        I_xx = (1/12) * m * (3*(r**2) + l**2)
        I_yy = (1/12) * m * (3*(r**2) + l**2)
        I_zz = (1/2)  * m * (r**2)

        return np.array([
            [I_xx, 0,    0   ],
            [0,    I_yy, 0   ],
            [0,    0,    I_zz]
        ])

    def config_mat(i) -> np.ndarray:
        M = np.identity(4)
        match i:
            case 0:
                M = np.array([ # M_{1,0}
                    [1, 0,  0, -L_1x],
                    [0, 0, -1,  0   ],
                    [0, 1,  0, -L_1y],
                    [0,  0,  0,     1]
                ])
            case 1:
                M = np.array([ # M_{2,1}
                    [1,  0,  0, -L_2x],
                    [0, -1,  0,     0],
                    [0,  0, -1, -L_2y],
                    [0,  0,  0,     1]
                ])
            case 2:
                M = np.array([ # M_{3,2}
                    [1, 0, 0,    0],
                    [0, 1, 0,    0],
                    [0, 0, 1, L_3z],
                    [0, 0, 0,    1]
                ])
            case 3:
                M = np.array([ # M_{4,3}
                    [1, 0, 0, L_4x],
                    [0, 1, 0,    0],
                    [0, 0, 1, L_4z],
                    [0, 0, 0,    1]
                ])
            case 4:
                M = np.array([ # M_{5,4}
                    [1, 0, 0, L_5x],
                    [0, 1, 0,    0],
                    [0, 0, 1, L_5z],
                    [0, 0, 0,    1]
                ])
            case 5:
                M = np.array([ # M_{6,5}
                    [1, 0, 0, L_6x],
                    [0, 1, 0,    0],
                    [0, 0, 1, L_6z],
                    [0, 0, 0,    1]
                ])
            case 6:
                M = np.array([ # M_{7,6}
                    [1, 0, 0, -2 * L_6x],
                    [0, 1, 0,         0],
                    [0, 0, 1,      L_7z],
                    [0, 0, 0,         1]
                ])
            case 7:
                M = np.array([ # M_{8,7}
                    [1, 0, 0, 2 * L_6x],
                    [0, 1, 0,        0],
                    [0, 0, 1,     L_8z],
                    [0, 0, 0,        1]
                ])

        return M

    def screw_axis(i) -> np.ndarray:
        A = np.array([0, 0, 0, 0, 0, 0])

        match i:
            case 0: # A_1
                A = np.array([1, 0, 0, 0, 0, 0])
            case 1: # A_2
                A = np.array([0, 1, 0, 0, 0, 0])
            case 2: # A_3
                A = np.array([0, 0, 1, 0, 0, 0])
            case 3: # A_4
                A = np.array([-1, 0, 0, 0, 0, 0])
            case 4: # A_5
                A = np.array([0, 0, 1, 0, 0, 0])
            case 5: # A_6
                A = np.array([-1, 0, 0, 0, 0, 0])
            case 6: # A_7
                A = np.array([0, 0, 1, 0, 0, 0])

        return A
    
    def spatial_inertia_cyl(m, r, l) -> np.ndarray:
        return np.block([
            [ inertia_cyl(m, r, l), np.zeros((3,3)) ],
            [ np.zeros((3,3))     , m * np.identity(3)]
        ])
    

    # --- SOLVER ---------------------------------------------------------------
    pos = [
        0,
        0,
        0,

        0,

        0,
        0,
        0
    ]

    vel = [
        0,
        0,
        0,

        0,

        0,
        0,
        0
    ]

    for i in range(0, 7):
        A_i = screw_axis(i)
        M_i = config_mat(i)
        T_i = trans(A_i, pos[i]) @ M_i

        V_i = 0