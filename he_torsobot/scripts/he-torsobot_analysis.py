import numpy as np

# --- vector and matrix handling -------------------------------------------------------------------
def unit(v: np.ndarray) -> np.ndarray:
    """
    Takes in a vector and ensures that it is a unit vector.
    """
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
    v = np.asarray(v).ravel()
    if v.shape != (3,):
        raise ValueError("Input vector must have exactly 3 elements.")
        
    v1, v2, v3 = v[0], v[1], v[2]
    
    return np.array([
        [-(v2**2 + v3**2),  v1 * v2,          v1 * v3        ],
        [  v1 * v2,        -(v1**2 + v3**2),  v2 * v3        ],
        [  v1 * v3,         v2 * v3,         -(v1**2 + v2**2)]
    ])


# --- rotation matrix via rodrigues formula --------------------------------------------------------
def rot(w: np.ndarray, theta: float) -> np.ndarray:
    if w.shape != (3,):
        raise ValueError("Input axis must have exactly 3 elements.")
    
    w_unit = unit(w)
    return np.identity(3) + np.sin(theta)*skew(w_unit) + (1-np.cos(theta))*skew2(w_unit)


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

def ad_rep(T: np.ndarray) -> np.ndarray:
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    pR = skew(p) @ R

    return np.block([
        [R,   np.zeros((3, 3))],
        [pR,  R               ]
    ])


# --- MAIN -----------------------------------------------------------------------------------------
if __name__ == "__main__":
    L1, L2 = 0.3, 0.3

    thetas = np.array([np.pi/3, np.pi/6, 0, 0, 0, 0, 0])
    screws = [
        screw(np.array([ 1,  0,  0]), np.array([0, 0, 0  ])), # S1
        screw(np.array([ 0, -1,  0]), np.array([0, 0, 0  ])), # S2
        screw(np.array([ 0,  0, -1]), np.array([0, 0, 0  ])), # S3
        screw(np.array([ 1,  0,  0]), np.array([0, 0, -L1])), # S4
        screw(np.array([ 0,  0, -1]), np.array([0, 0, -L2])), # S5
        screw(np.array([ 1,  0,  0]), np.array([0, 0, 0  ])), # S6
        screw(np.array([ 0,  0, -1]), np.array([0, 0, 0  ]))  # S7
    ]
    M = np.array([
        [ 1,  0,  0,  0       ],
        [ 0, -1,  0,  0       ],
        [ 0,  0, -1, -(L1+L2) ],
        [ 0,  0,  0,  1       ]
    ])

    T = PoE_space(screws, thetas, M)