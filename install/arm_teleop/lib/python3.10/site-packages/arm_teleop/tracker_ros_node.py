import threading
import queue
import sys
import os
import time

import numpy as np
import cv2
import rclpy

# tracking.py must be in the same directory as this file
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tracking import (
    build_camera_matrix, make_detector, detect_markers,
    BandTracker, wrist_rotation_matrix, compute_joint_angles,
    draw_axis, draw_arm_overlay, draw_hud, do_initialize,
    SHOULDER_IDS, ELBOW_IDS, WRIST_IDS,
    CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY,
)
from joint_publisher import JointPublisher


def tracker_thread(angle_queue, stop_event):

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera.")
        stop_event.set()
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    ret, frame = cap.read()
    if not ret:
        print("Cannot read from camera.")
        stop_event.set()
        return

    h, w     = frame.shape[:2]
    K, dist  = build_camera_matrix(w, h, CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY)
    detector_params  = make_detector()

    shoulder_tr = BandTracker(SHOULDER_IDS)
    elbow_tr    = BandTracker(ELBOW_IDS)
    wrist_tr    = BandTracker(WRIST_IDS)

    initialized      = False
    init_state       = {}
    joint_angles     = None
    countdown_active = False
    countdown_start  = None
    COUNTDOWN_SECS   = 3

    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            break

        detected     = detect_markers(frame, detector_params , K, dist)
        shoulder_pos = shoulder_tr.update(detected)
        elbow_pos    = elbow_tr.update(detected)
        wrist_pos    = wrist_tr.update(detected)
        wrist_R      = wrist_rotation_matrix(detected)

        for mid, data in detected.items():
            draw_axis(frame, K, dist, data['rvec'], data['tvec'])
            cx, cy = data['center_px'].astype(int)
            cv2.putText(frame, f"ID{mid}", (cx-15, cy-14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        draw_arm_overlay(frame, shoulder_pos, elbow_pos, wrist_pos, K)

        # countdown to initialization
        countdown_display = None
        if countdown_active:
            elapsed   = time.time() - countdown_start
            remaining = int(np.ceil(COUNTDOWN_SECS - elapsed))
            if elapsed >= COUNTDOWN_SECS:
                countdown_active = False
                state = do_initialize(shoulder_pos, elbow_pos, wrist_pos, wrist_R)
                if state:
                    init_state   = state
                    initialized  = True
                    joint_angles = None
                else:
                    missing = [n for n, p in [('shoulder', shoulder_pos),
                                               ('elbow',    elbow_pos),
                                               ('wrist',    wrist_pos)]
                               if p is None]
            else:
                countdown_display = remaining

        # calcualte and publish joint angles
        if initialized and all(p is not None for p in
                               [shoulder_pos, elbow_pos, wrist_pos]):
            joint_angles = compute_joint_angles(
                shoulder_pos, elbow_pos, wrist_pos, wrist_R, init_state
            )
            try:
                angle_queue.put_nowait(joint_angles)
            except queue.Full:
                pass 

        draw_hud(frame, joint_angles if initialized else None,
                 initialized, len(detected), countdown_display)

        cv2.imshow("Arm Tracker", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' ') and not countdown_active:
            if all(p is not None for p in [shoulder_pos, elbow_pos, wrist_pos]):
                countdown_active = True
                countdown_start  = time.time()
            else:
                missing = [n for n, p in [('shoulder', shoulder_pos),
                                           ('elbow',    elbow_pos),
                                           ('wrist',    wrist_pos)]
                           if p is None]

        elif key in (ord('r'), ord('R')):
            initialized = False
            countdown_active = False
            init_state = {}
            joint_angles = None
            shoulder_tr.reset()
            elbow_tr.reset()
            wrist_tr.reset()

        elif key in (ord('q'), 27):
            stop_event.set()
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    rclpy.init()

    angle_queue = queue.Queue(maxsize=2)
    stop_event  = threading.Event()

    # tracker runs in a background thread
    t = threading.Thread(
        target=tracker_thread,
        args=(angle_queue, stop_event),
        daemon=True,
    )
    t.start()

    node = JointPublisher(angle_queue)

    try:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop_event.set()


if __name__ == '__main__':
    main()