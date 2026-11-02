import json
import numpy as np

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']

# Longitudes de links obtenidas del modelo so101_new_calib.xml, esta en la carpeta models
l4, l5 = 0.028, 0.11257        # L1 = hipotenusa
FOREARM = (0.1349, 0.0052)     # l6
A = 0.0304                     # l0 + l2  
B = 0.1166                     # l1 + l3  
E = 0.1592                     # l7 + l8 

L1 = float(np.hypot(l4, l5))
alpha0 = float(np.arctan2(l4, l5))          
l6 = float(np.hypot(*FOREARM))
tilt = float(np.arctan2(FOREARM[1], FOREARM[0]))

# Si el brazo queda al reves se le cambia el signo a SIGN y se ajusta OFFSET
SIGN = np.array([-1.0, 1.0, 1.0, 1.0, 1.0])
_o2 = 2 * alpha0 - np.pi / 2   # con q_cal=0 el brazo queda vertical (forma de L del modelo)
OFFSET = np.array([0.0, _o2, -_o2 - tilt, tilt, 0.0])



def _to_array(q):
    if isinstance(q, dict):
        return np.array([q[j] for j in JOINTS], float)
    return np.asarray(q, float)[:5]


def cal_to_dh(q_cal_deg):
    return SIGN * np.radians(_to_array(q_cal_deg)) + OFFSET


def dh_to_cal(q_dh):
    return np.degrees((np.asarray(q_dh, float) - OFFSET) / SIGN)




# CINEMÁTICA DIRECTA 
def _A(a, al, d, th):
    ca, sa, ct, st = np.cos(al), np.sin(al), np.cos(th), np.sin(th)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0, sa, ca, d],
                     [0, 0, 0, 1]])



# forward kinematics
def fk(q_dh):
    
    q1, q2, q3, q4, q5 = q_dh
    rows = [(A, -np.pi / 2, B, q1),
            (-L1, 0, 0, q2 + (np.pi - alpha0)),
            (-l6, 0, 0, q3 + alpha0),
            (0, -np.pi / 2, 0, q4 + np.pi / 2),
            (0, np.pi, E, q5)]
    T = np.eye(4)
    for r in rows:
        T = T @ _A(*r)
    return T


# Debería dar puntos para graficar el brazo pero no funcionó 
def joint_points(q_dh):
    q1, q2, q3, q4, _ = q_dh
    d = q2 + q3 + q4
    plane = [(0.0, 0.0),
             (A, B),
             (A + l5 * np.cos(q2) + l4 * np.sin(q2), B + l4 * np.cos(q2) - l5 * np.sin(q2))]
    plane.append((plane[2][0] + l6 * np.cos(q2 + q3), plane[2][1] - l6 * np.sin(q2 + q3)))
    plane.append((plane[3][0] + E * np.cos(d), plane[3][1] - E * np.sin(d)))
    c, s = np.cos(q1), np.sin(q1)
    return np.array([[r * c, r * s, z] for r, z in plane])


# Devuelve el pitch q2+q3+q4
def fk_cal(q_cal_deg):
    q = cal_to_dh(q_cal_deg)
    p = joint_points(q)[-1]
    return p, float(np.degrees(q[1] + q[2] + q[3]))


# CINEMÁTICA INVERSA 
class Unreachable(ValueError):
    pass


# Solucion cerrada para la inverse kinematics
def ik_solutions(P, pitch, q5=0.0):
    x, y, z = P
    if np.hypot(x, y) < 1e-6:
        raise Unreachable("x = y = 0: q1 queda indeterminado")
    q1 = np.arctan2(y, x)
    r = x * np.cos(q1) + y * np.sin(q1)
    xp = r - A - E * np.cos(pitch)
    zp = z - B + E * np.sin(pitch)
    c3 = (xp ** 2 + zp ** 2 - L1 ** 2 - l6 ** 2) / (2 * L1 * l6)
    if abs(c3) > 1.0:
        raise Unreachable("punto fuera del espacio de trabajo (|cos q3| > 1)")
    sols = []
    for sgn in (+1.0, -1.0):
        q3p = sgn * np.arccos(c3)
        q3 = q3p - alpha0
        th1 = np.arctan2(-zp, xp) - np.arctan2(l6 * np.sin(q3p), L1 + l6 * np.cos(q3p))
        q2 = th1 + alpha0
        q4 = pitch - q2 - q3
        sols.append(np.array([q1, q2, q3, q4, q5]))
    return sols


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


#Determinar posiciones fuera de reach 
def ik_cal(P, pitch_deg, q5_deg=0.0, q_ref_cal=None, limits=None):
    sols = [dh_to_cal(s) for s in ik_solutions(P, np.radians(pitch_deg))]
    for s in sols:
        s[4] = q5_deg
        s[:] = (s + 180) % 360 - 180
    if limits is not None:
        sols = [s for s in sols if all(limits[j][0] <= v <= limits[j][1] for j, v in zip(JOINTS, s))]
    if not sols:
        raise Unreachable("ninguna solución dentro de los límites articulares")
    if q_ref_cal is not None:
        ref = _to_array(q_ref_cal)
        sols.sort(key=lambda s: np.sum(np.abs(_wrap(np.radians(s - ref)))))
    return dict(zip(JOINTS, sols[0]))


# Limitar rangos de joints pa no romper la vaina
def load_limits(json_path):
    with open(json_path) as f: #Esos rangos estan en el archivo calibration_files/so101_follower.json
        cal = json.load(f)
    lim = {}
    for j, c in cal.items():
        if j == 'gripper':
            lim[j] = (0.0, 100.0)
        else:
            half = (c['range_max'] - c['range_min']) / 2 * 360 / 4095
            lim[j] = (-half, half)
    return lim


# AUTOPRUEBA 
if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    rng = np.random.default_rng(1)
    # 1. FK por matrices DH 
    q = rng.uniform(-1, 1, 5)
    T = fk(q)
    q1, q2, q3, q4, _ = q
    Px = np.cos(q1) * (A + l6 * np.cos(q2 + q3) + l5 * np.cos(q2) + l4 * np.sin(q2) + E * np.cos(q2 + q3 + q4))
    Pz = B + l4 * np.cos(q2) - l5 * np.sin(q2) - l6 * np.sin(q2 + q3) - E * np.sin(q2 + q3 + q4)
    print("Reporte vs DH:", np.array([Px, Pz]), T[[0, 2], 3])
    print("Puntos vs DH :", joint_points(q)[-1], T[:3, 3])
    # 2. ida y vuelta FK -> IK -> FK
    worst = 0.0
    for _ in range(200):
        qc = np.array([rng.uniform(-60, 60), rng.uniform(-80, 20), rng.uniform(-10, 90), rng.uniform(-60, 60), 0])
        p, pitch = fk_cal(qc)
        try:
            sol = ik_cal(p, pitch, q_ref_cal=qc)
        except Unreachable:
            continue
        p2, _ = fk_cal(sol)
        worst = max(worst, np.linalg.norm(p2 - p))
    print(f"Error máximo FK(IK(FK(q))) vs FK(q): {worst:.2e} m")
    # 3. pose cero (brazo en L del modelo)
    print("Pose q_cal=0 -> posición, pitch:", fk_cal(np.zeros(5)))
