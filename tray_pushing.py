from ultralytics import YOLO
import cv2
import numpy as np
from pymycobot import MyCobot280
import time
import json

# Configurations 
MODEL_PATH = '/home/er/Desktop/yolo/mycobot_280_2/runs/detect/yolov8m_custom_run_2/weights/best.pt'
ROBOT_PORT = '/dev/ttyTHS1'
CAMERA_ID = 0
CALIBRATION_FILE = 'EyesInHand_matrix.json'
CAMERA_PARAMS_FILE = 'camera_params.npz'
POSITIONS_FILE = 'robot_positions.json'  

# Load in pre-recorded positions
print("Loading recorded robot positions...")
with open(POSITIONS_FILE, 'r') as f:
    recorded_positions = json.load(f)

# Extract the positions
ABOVE_TRAY = recorded_positions['above_tray']
PUSH_START = recorded_positions['push_start']
PUSH_END = recorded_positions['push_end']

print(f"Loaded positions:")
print(f"  Above tray: {ABOVE_TRAY}")
print(f"  Push start: {PUSH_START}")
print(f"  Push end: {PUSH_END}")

# Robot positions
HOME_POSITION = [0, 0, -90, 0, 0, 0]
OBSERVATION_POSITION = [0.00, 0.00, 0.00, -60, 0.00, 42.41]

# Load calibrations
print("Loading hand-eye calibration...")
with open(CALIBRATION_FILE, 'r') as f:
    hand_eye_matrix = np.array(json.load(f))
print("Hand-eye calibration matrix loaded:")
print(hand_eye_matrix)

# Load camera intrinsics
camera_params = np.load(CAMERA_PARAMS_FILE)
mtx = camera_params["mtx"]
dist = camera_params["dist"]


print("Loading YOLO model...")
model = YOLO(MODEL_PATH)

print("Connecting to robot...")
robot = MyCobot280(ROBOT_PORT, 1000000)
if not robot.is_power_on():
    robot.power_on()
    time.sleep(2)

print("Moving to home position...")
robot.send_angles(HOME_POSITION, 50)
time.sleep(5)

print("Opening camera...")
cap = cv2.VideoCapture(CAMERA_ID)
if not cap.isOpened():
    raise Exception("Cannot open camera")



def detect_tray_in_camera():
    """Detect tray and return its position in camera coordinates"""
    ret, frame = cap.read()
    if not ret:
        return None, None, None

    cv2.imwrite('camera_view.jpg', frame)
    print("Saved camera view to camera_view.jpg")
    
    # Run YOLO detection
    results = model.predict(source=frame, conf=0.5, verbose=False)
    result = results[0]
    
    if len(result.boxes) == 0:
        print("No tray detected")
        return None, None, frame
    
    # Get first detected tray
    box = result.boxes[0]
    coords = box.xyxy[0].cpu().numpy()
    confidence = float(box.conf)
    
    # Calculate center of bounding box in pixels
    center_x_px = int((coords[0] + coords[2]) / 2)
    center_y_px = int((coords[1] + coords[3]) / 2)
    
    print(f"Tray detected at pixel ({center_x_px}, {center_y_px}), confidence: {confidence:.2f}")
    
    # Draw on frame
    annotated = result.plot()
    cv2.circle(annotated, (center_x_px, center_y_px), 5, (0, 255, 0), -1)
    
    return True, confidence, annotated  # Just return that we detected it

# Main function to detect and push tray
def push_tray():
    print("\n=== Starting Tray Push with Recorded Positions ===")
    
    # Step 1: Move to observation position
    print("Moving to observation position...")
    robot.send_angles(OBSERVATION_POSITION, 50)
    time.sleep(5)
    
    # Step 2: Detect tray in camera
    print("Detecting tray...")
    detected, confidence, annotated = detect_tray_in_camera()
    
    if not detected:
        print("No tray found! Aborting.")
        return
    
    print(f"Tray detected with confidence: {confidence:.2f}")
    
    # Save detection
    cv2.imwrite('detected_tray.jpg', annotated)
    print("Saved detection to 'detected_tray.jpg'")
    
    # Step 3: Execute recorded motion sequence
    print("\n--- Executing Push Sequence ---")
    
    print("1. Moving above tray...")
    robot.send_angles(ABOVE_TRAY, 30)
    time.sleep(4)
    
    print("2. Moving to push start position (at tray edge)...")
    robot.send_angles(PUSH_START, 20)
    time.sleep(4)
    
    print("3. Pushing tray...")
    robot.send_angles(PUSH_END, 15)  
    time.sleep(5)
    
    print("4. Moving back above tray...")
    robot.send_angles(ABOVE_TRAY, 20)
    time.sleep(3)
    
    # Step 5: Return home
    print("5. Returning home...")
    robot.send_angles(HOME_POSITION, 50)
    time.sleep(5)
    
    print("\n=== Push Complete ===")


if __name__ == "__main__":
    try:
        push_tray()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if cap is not None:
            cap.release()
            print("Camera released")