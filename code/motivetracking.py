# -*- coding: utf-8 -*-
#require NatnetSDK!
"""
This code send the compensation degree needed to be rotate to the MCU board  
Motive → Arduino compensation to accumulated torque 
"""
import NatNetClient
import serial
import time
import threading
import numpy as np
from collections import deque
import csv
import os



# -------------------------------
STEP_ANGLE_DEG = 0.225 #depends on the A4988 conection 
GAIN = 1.0
RIGIDBODY_ID = 1
PORT_NAME = 'COM17'
BAUD_RATE = 115200
WINDOW_INTERVAL = 0.05
ANGLE_THRESHOLD = 90.0
MIN_COMMAND_INTERVAL = 0.2
MAX_STEPS_PER_CMD = 8000
BUSY_TIMEOUT = 2.0
DIRECTION_SIGN = 1
MAX_VALID_DELTA = 40.0


#SAVE_DIR = r"D:\rotationdataa"
#os.makedirs(SAVE_DIR, exist_ok=True)
#CSV_PATH = os.path.join(SAVE_DIR, "rotation_log.csv")


orientation_history = deque(maxlen=400)
last_command_time = 0.0
motor_busy = False
last_tx_started_at = 0.0
accum_angle = 0.0
ser = None
lock = threading.Lock()
q_prev = None

def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=float)

def quat_conj(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z], dtype=float)

def ensure_continuity(q, q_prev):
    if q_prev is not None and np.dot(q, q_prev) < 0:
        q = -q
    return q

def delta_yaw_yup(q_now, q_past):
    dq = quat_mul(q_now, quat_conj(q_past))
    w, x, y, z = dq
    v = np.array([x, y, z], dtype=float)
    nv = np.linalg.norm(v)
    if nv < 1e-12:
        return 0.0
    axis = v / nv
    angle = 2.0 * np.degrees(np.arctan2(nv, w))
    up = np.array([0, 1, 0])
    sign = np.sign(np.dot(axis, up))
    return float(angle * (1.0 if sign >= 0 else -1.0))

# -------------------------------
# 串口初始化 + 回饋監聽
# -------------------------------
def setup_serial():
    global ser
    try:
        ser = serial.Serial(PORT_NAME, BAUD_RATE, timeout=0.05)
        print(f"[OK] Serial port {PORT_NAME} opened.")
        time.sleep(2.0)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"[ERROR] Serial connection failed: {e}")
        ser = None

def read_serial_feedback():
    global motor_busy, accum_angle, last_command_time
    while True:
        try:
            if ser and ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"[RX] {line}")
                    if "DONE" in line:
                        with lock:
                            motor_busy = False
                        t = time.time()
                        #csv_writer.writerow([f"{t:.3f}", "", f"{accum_angle:.3f}", "False", "DONE"])
                        print("[INFO] Motor DONE 回報 → 解除忙碌狀態")
                        if abs(accum_angle) >= ANGLE_THRESHOLD:
                            print("[AUTO] 檢測到新累積角 → 立即再補償一次")
                            trigger_motor_command()
        except Exception as e:
            print(f"[ERROR] Serial read failed: {e}")
        time.sleep(0.01)

# -------------------------------
# CSV 初始化
# -------------------------------
#csv_file = open(CSV_PATH, 'w', newline='')
#csv_writer = csv.writer(csv_file)
#csv_writer.writerow(["timestamp", "yaw_delta_deg", "accum_angle_deg", "motor_busy", "event"])

# -------------------------------
# 傳送補償指令
# -------------------------------
def trigger_motor_command():
    global accum_angle, motor_busy, last_tx_started_at, last_command_time
    t = time.time()
    dt = t - last_command_time
    if dt < MIN_COMMAND_INTERVAL or motor_busy:
        return

    steps = int(abs(accum_angle) / STEP_ANGLE_DEG)
    steps = min(steps, MAX_STEPS_PER_CMD)
    direction = DIRECTION_SIGN * (1 if accum_angle > 0 else -1)

    # degree calculate
    compensated_deg = steps * STEP_ANGLE_DEG

    try:
        msg = f"{direction},{steps}\n"
        ser.reset_input_buffer()
        ser.write(msg.encode('utf-8'))
        with lock:
            motor_busy = True
        last_tx_started_at = t
        last_command_time = t

        print(f"[TX] 累積={accum_angle:+.2f}° → steps={steps}, dir={direction}, 補償={compensated_deg:.2f}°")
        #csv_writer.writerow([
        #    f"{t:.3f}", "", f"{accum_angle:.3f}", "True",
        #    f"TRIGGER dir={direction}, steps={steps}, compensated={compensated_deg:.2f}°"
        #])

        # 保留剩餘角度（只扣掉閾值）
        accum_angle -= np.sign(accum_angle) * ANGLE_THRESHOLD

    except Exception as e:
        print(f"[ERROR] Serial write failed: {e}")

# -------------------------------
# Motive callback
# -------------------------------
def process_rigidbody(rigid_body_id, position, orientation):
    global accum_angle, q_prev, motor_busy, last_tx_started_at

    if rigid_body_id != RIGIDBODY_ID:
        return
    if ser is None or not ser.is_open:
        return

    t = time.time()

    ox, oy, oz, ow = orientation
    q = np.array([ow, ox, oy, oz], dtype=float)
    q = q / np.linalg.norm(q)
    q = ensure_continuity(q, q_prev)

    if q_prev is None:
        q_prev = q
        return

    delta_yaw = delta_yaw_yup(q, q_prev) * GAIN
    q_prev = q

    if abs(delta_yaw) > MAX_VALID_DELTA:
        return

    accum_angle += delta_yaw
    accum_angle = float(np.clip(accum_angle, -360.0, 360.0))

    print(f"[DBG] busy={motor_busy}, Δyaw={delta_yaw:+.2f}°, 累積={accum_angle:+.2f}°")
    #csv_writer.writerow([f"{t:.3f}", f"{delta_yaw:.3f}", f"{accum_angle:.3f}", str(motor_busy), "NONE"])

    if motor_busy and (t - last_tx_started_at) > BUSY_TIMEOUT:
        with lock:
            motor_busy = False
        print(f"[WDOG] timeout → motor freed")

    if abs(accum_angle) >= ANGLE_THRESHOLD and not motor_busy:
        trigger_motor_command()

# -------------------------------
# 主程式
# -------------------------------
if __name__ == "__main__":
    print("=== Motive → Arduino 累積補償控制 (顯示補償角度) ===")
    setup_serial()
    threading.Thread(target=read_serial_feedback, daemon=True).start()

    client = NatNetClient.NatNetClient()
    client.rigid_body_listener = process_rigidbody
    client.run()

    print("[Working] Streaming from Motive... Ctrl+C 結束")
    try:
        while True:
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[STOP] User interrupt.")
    finally:
        client.shutdown()
        if ser:
            ser.close()
        #csv_file.close()
        #print(f"[Saved] CSV log → {os.path.abspath(CSV_PATH)}")
