

import cv2
import numpy as np
import time
from collections import deque


SHOULDER_IDS  = {0, 1, 2, 3}
ELBOW_IDS     = {4, 5, 6, 7}
WRIST_IDS     = {8, 9, 10, 11}

MARKER_LENGTH = 0.04        #  marker size in meters
ARUCO_DICT    = cv2.aruco.DICT_4X4_50


SMOOTH_FRAMES = 5

CAMERA_FX = None
CAMERA_FY = None
CAMERA_CX = None
CAMERA_CY = None



def build_camera_matrix(frame_w, frame_h, fx=None, fy=None, cx=None, cy=None):
    if fx is None:
        fx = fy = frame_w
        cx = frame_w  / 2.0
        cy = frame_h / 2.0
    K    = np.array([[fx,  0, cx],
                     [ 0, fy, cy],
                     [ 0,  0,  1]], dtype=np.float64)
    dist = np.zeros((4, 1), dtype=np.float64)
    return K, dist



def make_detector():

    params     = cv2.aruco.DetectorParameters_create()

    params.adaptiveThreshWinSizeMin  = 3
    params.adaptiveThreshWinSizeMax  = 53
    params.adaptiveThreshWinSizeStep = 10
    params.adaptiveThreshConstant    = 7

    params.minMarkerPerimeterRate = 0.02   # lowered from default, 0.03 was not sensitive enough

    params.cornerRefinementMethod        = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize       = 5
    params.cornerRefinementMaxIterations = 30
    params.cornerRefinementMinAccuracy   = 0.1

    return params


def preprocess_frame(frame):
    # somebasic prepprocessing 
    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    return clahe.apply(gray)


def detect_markers(frame, detector_params, camera_matrix, dist_coeffs):
    gray    = preprocess_frame(frame)
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=detector_params)    # detects hte markers that can be seen to estimate a pose

    results = {}
    if ids is None:
        return results

    for i, marker_id in enumerate(ids.flatten()):
        c = corners[i]
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            c, MARKER_LENGTH, camera_matrix, dist_coeffs
        )
        results[int(marker_id)] = {
            'rvec':      rvec[0][0],
            'tvec':      tvec[0][0],
            'center_px': c[0].mean(axis=0),
        }
    return results


class BandTracker:

    MAX_HOLD_FRAMES = 15   # how long to hold the position for if no marker detected

    def __init__(self, band_ids, smooth_frames=SMOOTH_FRAMES):
        self.band_ids      = band_ids
        self.history       = deque(maxlen=smooth_frames)
        self.frames_unseen = 0
        self.last_pos      = None

    def update(self, detected):
        positions = [detected[mid]['tvec']
                     for mid in self.band_ids if mid in detected]
        if positions:
            self.history.append(np.mean(positions, axis=0))
            self.frames_unseen = 0
            self.last_pos = np.mean(self.history, axis=0)
        else:
            self.frames_unseen += 1
            if self.frames_unseen > self.MAX_HOLD_FRAMES:
                self.last_pos = None
        return self.last_pos

    def reset(self):
        self.history.clear()
        self.frames_unseen = 0
        self.last_pos      = None


def wrist_rotation_matrix(detected):
    
    best_R, best_score = None, -1.0
    for mid in WRIST_IDS:
        if mid not in detected:
            continue
        R, _ = cv2.Rodrigues(detected[mid]['rvec'])
        score = -np.dot(R[:, 2], np.array([0.0, 0.0, 1.0]))
        if score > best_score:
            best_score, best_R = score, R
    return best_R

#geometry  helper functions
def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-6 else np.zeros(3)

def angle_between(a, b):
    return np.arccos(np.clip(np.dot(unit(a), unit(b)), -1.0, 1.0))

def signed_angle(v1, v2, axis):
    v1, v2, axis = unit(v1), unit(v2), unit(axis)
    return np.arctan2(np.dot(np.cross(v1, v2), axis), np.dot(v1, v2))


def compute_joint_angles(shoulder_pos, elbow_pos, wrist_pos, wrist_R, init_state):

    upper_arm = elbow_pos - shoulder_pos
    forearm   = wrist_pos - elbow_pos
    init_ua   = init_state['upper_arm_vec']
    init_fa   = init_state['forearm_vec']

    # Angle between upper_arm and forearm. 0 at init
    elbow_flex = angle_between(upper_arm, forearm) - angle_between(init_ua, init_fa)

    # shoulder 
    world_up    = np.array([0.0, -1.0, 0.0])
    abduct_axis = unit(np.cross(unit(init_ua), world_up))
    flex_axis   = unit(np.cross(world_up, unit(init_ua)))

    shoulder_abduct = signed_angle(init_ua, upper_arm, abduct_axis)
    shoulder_flex   = signed_angle(init_ua, upper_arm, flex_axis)

    # forearm
    # project wrist band marker Y axis perpendicular to the forearm vector, compare to direction stored at init
    def perp_to(v, ax):
        return unit(v - np.dot(v, ax) * ax)

    forearm_pronate = 0.0
    if wrist_R is not None and init_state.get('forearm_ref_up') is not None:
        fa_unit  = unit(forearm)
        cur_perp = perp_to(wrist_R[:, 1],                fa_unit)
        ref_perp = perp_to(init_state['forearm_ref_up'], fa_unit)
        if np.linalg.norm(cur_perp) > 0.1 and np.linalg.norm(ref_perp) > 0.1:
            forearm_pronate = signed_angle(ref_perp, cur_perp, fa_unit)

    return {
        'shoulder_flex':   float(shoulder_flex),
        'shoulder_abduct': float(shoulder_abduct),
        'elbow_flex':      float(elbow_flex),
        'forearm_pronate': float(forearm_pronate),
    }




def draw_axis(frame, K, dist, rvec, tvec, length=0.03):
    cv2.drawFrameAxes(frame, K, dist, rvec, tvec, length)


def project_pt(pt3d, K):
    return (int(pt3d[0]/pt3d[2]*K[0,0] + K[0,2]),
            int(pt3d[1]/pt3d[2]*K[1,1] + K[1,2]))


def draw_arm_overlay(frame, shoulder_pos, elbow_pos, wrist_pos, K):
    if shoulder_pos is not None and elbow_pos is not None:
        cv2.line(frame, project_pt(shoulder_pos, K),
                 project_pt(elbow_pos, K), (0, 255, 0), 3)
    if elbow_pos is not None and wrist_pos is not None:
        cv2.line(frame, project_pt(elbow_pos, K),
                 project_pt(wrist_pos, K), (0, 200, 255), 3)
    for pos, label, color in [(shoulder_pos, 'A', (0, 255, 0)),
                               (elbow_pos,   'B', (0, 200, 255)),
                               (wrist_pos,   'C', (255, 100, 0))]:
        if pos is not None:
            p = project_pt(pos, K)
            cv2.circle(frame, p, 10, color, -1)
            cv2.putText(frame, label, (p[0]+12, p[1]+6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)


def draw_hud(frame, angles, initialized, markers_visible, countdown=None):
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (370, 225), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

    def d(r): return np.degrees(r)

    if countdown is not None:
        status = f"initialize in {countdown}s"
        color  = (0, 140, 255)
    elif initialized:
        status = "tracking"
        color  = (0, 255, 100)
    else:
        status = "press space to initalize"
        color  = (0, 180, 255)

    lines = [f"markers visible : {markers_visible}",
             f"status          : {status}"]

    if angles:
        p_str = (f"{d(angles['forearm_pronate']):+.1f}"
                 if angles['forearm_pronate'] != 0.0 else " N/A")
        lines += [
            f"shoulder flex   : {d(angles['shoulder_flex']):+.1f} deg",
            f"shoulder abduct : {d(angles['shoulder_abduct']):+.1f} deg",
            f"elbow flex      : {d(angles['elbow_flex']):+.1f} deg",
            f"forearm pronate : {p_str} deg",
        ]

    for i, line in enumerate(lines):
        cv2.putText(frame, line, (18, 38+i*28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)


def do_initialize(shoulder_pos, elbow_pos, wrist_pos, wrist_R):
    if any(p is None for p in [shoulder_pos, elbow_pos, wrist_pos]):
        return None
    ua = elbow_pos - shoulder_pos
    fa = wrist_pos - elbow_pos
    return {
        'shoulder_pos':   shoulder_pos.copy(),
        'elbow_pos':      elbow_pos.copy(),
        'wrist_pos':      wrist_pos.copy(),
        'upper_arm_vec':  ua.copy(),
        'forearm_vec':    fa.copy(),
        'upper_arm_len':  float(np.linalg.norm(ua)),
        'forearm_len':    float(np.linalg.norm(fa)),
        'forearm_ref_up': wrist_R[:, 1].copy() if wrist_R is not None else None,
    }
