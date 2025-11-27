import os
os.environ["GLOG_minloglevel"] = "2"
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"

import time
import cv2
import numpy as np
import mediapipe as mp
from absl import logging as absl_logging
import serial, serial.tools.list_ports

# hide mediapipe logs
absl_logging.set_verbosity(absl_logging.ERROR)

# =========================
# Config (tune to taste)
# =========================
EMA_ALPHA = 0.6            # smoothing for y position
VEL_DT_MIN = 1/60          # clamp dt for velocity

# Position thresholds (normalized; + up, - down)
JUMP_Y_THRESH = +0.40      # up movement is a jump
DUCK_Y_THRESH = -0.40      # down movement is a duck

# Dwell frames to confirm states (prevents jitter)
JUMP_FRAMES = 2
DUCK_FRAMES = 2
IDLE_FRAMES = 2

COOLDOWN_MS = 200          # minimum time between commands
BASELINE_DRIFT_BETA = 0.002 # slow recenter when idle
CALIBRATION_HOLD_MS = 900  # open-palm hold to calibrate

HUD_SCALE = 220
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# =========================
# Serial (UART) helpers
# =========================
SER_BAUD = 115200
ser = None  # global handle for serial port

def open_uart(port_hint: str = None):
    # Try to open the serial port (FTDI FT232RL)
    global ser
    if ser and ser.is_open:
        return ser

    candidates = []
    # look for FTDI or USB-Serial devices
    for p in serial.tools.list_ports.comports():
        name = (p.device or "")
        desc = (p.description or "").lower()
        manu = (p.manufacturer or "").lower()
        if ("usbserial" in name.lower()) or ("ftdi" in manu) or ("ftdi" in desc) or ("usb serial" in desc):
            candidates.append(name)

    port = port_hint or (candidates[0] if candidates else None)
    if not port:
        # last resort on macOS, prefer /dev/cu.*
        for p in serial.tools.list_ports.comports():
            if p.device.startswith("/dev/cu."):
                port = p.device
                break

    if not port:
        print("⚠️ No serial device found. Plug in the SH-U09C.")
        return None

    try:
        ser = serial.Serial(
            port, 
            SER_BAUD, 
            timeout=0,
            write_timeout=0.1,  # prevent blocking on write
            rtscts=False,       # disable flow control
            dsrdtr=False,       # disable flow control
            exclusive=True      # lock port
        )
        
        # set DTR/RTS low to prevent FPGA reset
        ser.dtr = False
        ser.rts = False
        
        # important: let the chip chill
        time.sleep(0.3)
        
        # try to set FTDI latency timer to 1ms
        try:
            import fcntl
            USBDEVFS_SETLATENCY = 0x5609
            latency = 1
            fcntl.ioctl(ser.fileno(), USBDEVFS_SETLATENCY, latency)
            print(f"✅ Set FTDI latency to {latency}ms")
        except:
            pass
        
        # send IDLE to wake up the connection
        ser.write(b'I')
        ser.flush()
        time.sleep(0.05)
        
        print(f"✅ UART open @ {port} {SER_BAUD} 8N1 (FTDI FT232RL)")
        print(f"   DTR={ser.dtr} RTS={ser.rts}")
        return ser
    except Exception as e:
        print(f"⚠️ UART open failed on {port}: {e}")
        ser = None
        return None

MAP = {'JUMP': b'J', 'DUCK': b'D', 'IDLE': b'I'}

def send_event(evt: str):
    # send command byte over UART
    b = MAP.get(evt)
    if not b:
        return
    try:
        if ser and ser.is_open:
            ser.write(b)
            ser.flush()          # force send now
            time.sleep(0.025)    # wait for byte to transmit
    except Exception as e:
        print(f"⚠️ UART write error: {e}")

# =========================
# Pure-OpenCV camera picker
# =========================
def select_mac_camera_index(start_idx=0, max_idx=6, window_name="Select Camera ([ / ] to switch, Enter to use)"):
    # pick camera index on macOS
    def open_cam(idx):
        cap = cv2.VideoCapture(idx, cv2.CAP_AVFOUNDATION)
        if not cap.isOpened():
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        return cap

    idx = start_idx
    cap = open_cam(idx)
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    while True:
        frame_ok, frame = (False, None) if cap is None else cap.read()
        if frame_ok:
            frame = cv2.flip(frame, 1)
        else:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            frame[:] = (30, 30, 30)

        label = f"Index {idx}  ({'OK' if frame_ok else 'NO SIGNAL'})"
        cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        cv2.putText(frame, "Use [ and ] to switch. Press Enter to select. (q to quit)",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220,220,220), 1)

        cv2.imshow(window_name, frame)
        k = cv2.waitKey(1) & 0xFF

        if k in (ord('q'), 27):
            if cap: cap.release()
            cv2.destroyWindow(window_name)
            return idx if cap is not None else start_idx
        elif k == ord(']'):
            if cap: cap.release()
            idx = (idx + 1) % max_idx
            cap = open_cam(idx)
        elif k == ord('['):
            if cap: cap.release()
            idx = (idx - 1) % max_idx
            cap = open_cam(idx)
        elif k in (13, 10):  # Enter
            if cap: cap.release()
            cv2.destroyWindow(window_name)
            return idx

# =========================
# MediaPipe setup
# =========================
mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

POSE = mp_pose.Pose(
    static_image_mode=False,
    model_complexity=1,
    enable_segmentation=False,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)
HANDS = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# =========================
# Helpers
# =========================
def now_ms():
    # get current time in ms
    return int(time.time() * 1000)

def ema(prev, curr, alpha):
    # exponential moving average for smooth movement
    return alpha * curr + (1 - alpha) * prev if prev is not None else curr

def get_landmarks_normalized(results_pose):
    # get nose, shoulders, hips landmarks (0 to 1 normalized coords)
    if not results_pose or not results_pose.pose_landmarks:
        return None
    lm = results_pose.pose_landmarks.landmark
    try:
        nose = lm[mp_pose.PoseLandmark.NOSE]
        l_sh = lm[mp_pose.PoseLandmark.LEFT_SHOULDER]
        r_sh = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER]
        l_hip = lm[mp_pose.PoseLandmark.LEFT_HIP]
        r_hip = lm[mp_pose.PoseLandmark.RIGHT_HIP]
        return {
            "nose": (nose.x, nose.y),
            "l_sh": (l_sh.x, l_sh.y),
            "r_sh": (r_sh.x, r_sh.y),
            "l_hip": (l_hip.x, l_hip.y),
            "r_hip": (r_hip.x, r_hip.y),
        }
    except:
        return None

def body_scale(pts):
    # vertical distance between shoulders and hips
    sh_y = (pts["l_sh"][1] + pts["r_sh"][1]) / 2.0
    hip_y = (pts["l_hip"][1] + pts["r_hip"][1]) / 2.0
    return abs(hip_y - sh_y)

def count_open_palm_above_head(hands_result, head_y_norm):
    # check for a hand held up (for calibration)
    if not hands_result or not hands_result.multi_hand_landmarks:
        return False
    FINGERS = [
        (mp_hands.HandLandmark.INDEX_FINGER_TIP, mp_hands.HandLandmark.INDEX_FINGER_PIP),
        (mp_hands.HandLandmark.MIDDLE_FINGER_TIP, mp_hands.HandLandmark.MIDDLE_FINGER_PIP),
        (mp_hands.HandLandmark.RING_FINGER_TIP, mp_hands.HandLandmark.RING_FINGER_PIP),
        (mp_hands.HandLandmark.PINKY_TIP, mp_hands.HandLandmark.PINKY_PIP),
    ]
    for hand_lms in hands_result.multi_hand_landmarks:
        extended = 0
        for tip_idx, pip_idx in FINGERS:
            tip = hand_lms.landmark[tip_idx]
            pip = hand_lms.landmark[pip_idx]
            # finger is extended if tip is higher than pip
            if tip.y < pip.y:
                extended += 1
        wrist = hand_lms.landmark[mp_hands.HandLandmark.WRIST]
        # check if palm is mostly open and above the nose/head
        if extended >= 4 and (wrist.y < head_y_norm):
            return True
    return False

# ----- CLAMPED HUD -----
def draw_hud(frame, y0_pix_raw, yhat, jump_thr, duck_thr, state):
    # draw the vertical status bar
    h, w = frame.shape[:2]
    cx = 30

    # clip the bar to the screen
    margin = 10
    half = HUD_SCALE // 2
    y0_pix = int(np.clip(y0_pix_raw, half + margin, h - half - margin))

    # draw the vertical bar and baseline circle
    top = int(y0_pix - half)
    bot = int(y0_pix + half)
    cv2.line(frame, (cx, top), (cx, bot), (200, 200, 200), 2)
    cv2.circle(frame, (cx, y0_pix), 3, (180, 180, 180), -1)

    # draw jump and duck thresholds
    j_pix = int(y0_pix - jump_thr * HUD_SCALE)
    d_pix = int(y0_pix - duck_thr * HUD_SCALE)
    j_pix = int(np.clip(j_pix, 0, h - 1))
    d_pix = int(np.clip(d_pix, 0, h - 1))

    cv2.line(frame, (cx-10, j_pix), (cx+10, j_pix), (0, 255, 0), 2)   # JUMP (green)
    cv2.line(frame, (cx-10, d_pix), (cx+10, d_pix), (0, 0, 255), 2)   # DUCK (red)
    cv2.putText(frame, "JUMP", (cx+14, j_pix+5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0), 1)
    cv2.putText(frame, "DUCK", (cx+14, d_pix+5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,0,255), 1)

    # draw current position dot
    curr_pix = int(y0_pix - yhat * HUD_SCALE)
    curr_pix = int(np.clip(curr_pix, 0, h - 1))
    cv2.circle(frame, (cx, curr_pix), 5, (255, 255, 255), -1)

    # display state
    cv2.putText(frame, f"State: {state}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

# =========================
# Main
# =========================
def main():
    chosen_idx = select_mac_camera_index(start_idx=0, max_idx=6)
    cap = cv2.VideoCapture(chosen_idx, cv2.CAP_AVFOUNDATION)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {chosen_idx}.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    window_title = "Online Dinosaur Game (Jump/Duck/Idle)"
    cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)

    # connect to serial port
    open_uart()

    # state variables
    baseline_locked = False
    baseline_y0 = None
    scale_S = None
    yhat = None
    prev_yhat = None
    prev_t = time.time()

    last_fire_ms = 0
    state = "IDLE"
    last_emitted = None

    jump_streak = 0
    duck_streak = 0
    idle_streak = 0
    palm_hold_start_ms = None

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            frame = cv2.flip(frame, 1)
            h, w = frame.shape[:2]

            # run pose and hand tracking
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pose_res = POSE.process(frame_rgb)
            hands_res = HANDS.process(frame_rgb)

            if pose_res and pose_res.pose_landmarks:
                # draw pose landmarks
                mp_drawing.draw_landmarks(
                    frame, pose_res.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=mp_drawing.DrawingSpec(color=(0,255,255), thickness=2, circle_radius=2),
                    connection_drawing_spec=mp_drawing.DrawingSpec(color=(128,128,0), thickness=2)
                )

            pts = get_landmarks_normalized(pose_res)

            if pts:
                nose_y = pts["nose"][1]
                S_now = body_scale(pts)

                if baseline_locked and scale_S is not None and scale_S > 1e-4:
                    # calculate normalized displacement (+ up, - down)
                    y_norm = (baseline_y0 - nose_y) / scale_S
                    # smooth the movement
                    yhat = ema(yhat, y_norm, EMA_ALPHA)

                    # calculate velocity (for debugging)
                    t = time.time()
                    dt = max(t - prev_t, VEL_DT_MIN)
                    prev_t = t
                    vy = 0.0 if prev_yhat is None else (yhat - prev_yhat) / dt
                    prev_yhat = yhat

                    tms = now_ms()
                    can_fire = (tms - last_fire_ms) >= COOLDOWN_MS

                    # check position against thresholds
                    is_jump = (yhat is not None and yhat >= JUMP_Y_THRESH)
                    is_duck = (yhat is not None and yhat <= DUCK_Y_THRESH)
                    is_idle = (yhat is not None and DUCK_Y_THRESH < yhat < JUMP_Y_THRESH)

                    # count frames in current state
                    jump_streak = jump_streak + 1 if is_jump else 0
                    duck_streak = duck_streak + 1 if is_duck else 0
                    idle_streak = idle_streak + 1 if is_idle else 0

                    fired = None
                    # check if we can trigger an event (must hold for N frames)
                    # duck has priority, then jump, then idle
                    if can_fire and duck_streak >= DUCK_FRAMES:
                        fired = "DUCK"
                    elif can_fire and jump_streak >= JUMP_FRAMES:
                        fired = "JUMP"
                    elif can_fire and idle_streak >= IDLE_FRAMES:
                        fired = "IDLE"

                    if fired:
                        state = fired
                        last_fire_ms = tms
                        if fired != last_emitted:
                            print(f"[{time.strftime('%H:%M:%S')}] {fired}")
                            send_event(fired)    # <-- SEND SERIAL COMMAND
                            last_emitted = fired

                    # slow drift baseline to prevent accumulating error
                    if state == "IDLE":
                        baseline_y0 = (1 - BASELINE_DRIFT_BETA) * baseline_y0 + BASELINE_DRIFT_BETA * nose_y

                    # draw status on screen
                    y0_pix_raw = int(baseline_y0 * h) if baseline_y0 is not None else int(h/2)
                    draw_hud(frame, y0_pix_raw, yhat if yhat is not None else 0.0,
                             JUMP_Y_THRESH, DUCK_Y_THRESH, state)
                    cv2.putText(frame, f"yhat={yhat:+.3f} vy={vy:+.2f}/s",
                                 (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,255), 1)
                    cv2.putText(frame, "Calibrated", (w-150, 30),
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

                else:
                    # not calibrated yet: use open palm above head to lock baseline
                    palm_up = count_open_palm_above_head(hands_res, head_y_norm=nose_y)
                    if palm_up:
                        if palm_hold_start_ms is None:
                            palm_hold_start_ms = now_ms()
                        elapsed = now_ms() - palm_hold_start_ms
                        pct = min(100, int(100 * elapsed / CALIBRATION_HOLD_MS))
                        cv2.putText(frame, f"Hold palm to calibrate... {pct}%",
                                     (10, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
                        if elapsed >= CALIBRATION_HOLD_MS:
                            # lock it in!
                            baseline_locked = True
                            baseline_y0 = nose_y
                            scale_S = max(S_now, 1e-4)
                            yhat = 0.0
                            prev_yhat = 0.0
                            prev_t = time.time()
                            palm_hold_start_ms = None
                            last_emitted = None
                            jump_streak = duck_streak = idle_streak = 0
                            print(">> Calibrated via open palm.")
                    else:
                        palm_hold_start_ms = None

                    cv2.putText(frame, "Raise OPEN PALM above head to calibrate (or press 'c')",
                                 (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)

            else:
                palm_hold_start_ms = None
                cv2.putText(frame, "No pose detected. Step back / improve lighting.",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')): # quit
                break
            if key == ord('c') and pts: # manual calibrate
                baseline_locked = True
                baseline_y0 = pts["nose"][1]
                scale_S = max(body_scale(pts), 1e-4)
                yhat = 0.0
                prev_yhat = 0.0
                prev_t = time.time()
                palm_hold_start_ms = None
                last_emitted = None
                jump_streak = duck_streak = idle_streak = 0
                print(">> Calibrated via key 'c'.")
            if key == ord('r'): # reset
                baseline_locked = False
                baseline_y0 = None
                scale_S = None
                yhat = None
                prev_yhat = None
                jump_streak = duck_streak = idle_streak = 0
                last_emitted = None
                print(">> Calibration reset.")

            cv2.imshow(window_title, frame)
    finally:
        # cleanup everything
        try:
            cap.release()
        except:
            pass
        try:
            cv2.destroyAllWindows()
        except:
            pass
        try:
            if ser and ser.is_open:
                ser.close()
                print("✅ UART closed")
        except:
            pass
        try:
            POSE.close()
            HANDS.close()
        except:
            pass

if __name__ == "__main__":
    main()
