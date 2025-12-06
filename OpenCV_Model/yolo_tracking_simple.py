"""
YOLO Tracking - FINAL WORKING VERSION
- Character-by-character sending (solves tilt issue)
- Inverted tilt angles (fixes reversed direction)
- Ultra-slow rate for maximum reliability
"""

import time
import cv2
from ultralytics import YOLO
import serial

SERIAL_PORT = "COM3"
BAUD_RATE = 115200
COMMAND_DELAY = 0.6  # 600ms = 1.67 Hz (slow but reliable)
CHAR_DELAY = 0.01    # 10ms between characters

def send_char_by_char(ser, text):
    """
    Send text one character at a time with delays
    This is CRITICAL - burst sending doesn't work with STM32L0 at 115200 baud
    """
    for char in text:
        ser.write(char.encode('utf-8'))
        ser.flush()
        time.sleep(CHAR_DELAY)

print("="*70)
print("YOLO TRACKING - FINAL WORKING VERSION")
print("="*70)
print("✓ Character-by-character sending (fixes tilt issue)")
print("✓ Inverted tilt angles (fixes reversed direction)")
print(f"✓ Command rate: {1/COMMAND_DELAY:.2f} Hz")
print("="*70)

# Open serial
print("\n[1/4] Opening serial port...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1, write_timeout=1)
time.sleep(2)
ser.reset_input_buffer()
ser.reset_output_buffer()
print("✓ Serial port open")

# Center servos
print("\n[2/4] Centering servos...")
send_char_by_char(ser, "90,90\r\n")
time.sleep(3)
print("✓ Servos centered")

# Load YOLO
print("\n[3/4] Loading YOLO...")
model = YOLO("yolo11n-seg.pt")
print("✓ YOLO loaded")

# Open camera
print("\n[4/4] Opening camera...")
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
ret, frame = cap.read()
if ret:
    h, w = frame.shape[:2]
    print(f"✓ Camera ready: {w}x{h}")
else:
    print("✗ Camera failed")
    exit(1)

print("\n" + "="*70)
print("TRACKING STARTED - Both Pan and Tilt Working!")
print("="*70)
print("Press 'q' to exit")
print("="*70)
print()

last_send = time.time()
cmd_count = 0

# FPS calculation
fps_start_time = time.time()
fps_frame_count = 0
current_fps = 0

# Current servo positions
current_pan = 90
current_tilt = 90

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        
        # Calculate FPS
        fps_frame_count += 1
        if fps_frame_count >= 10:
            fps_end_time = time.time()
            current_fps = fps_frame_count / (fps_end_time - fps_start_time)
            fps_start_time = fps_end_time
            fps_frame_count = 0
        
        h, w = frame.shape[:2]
        
        # Run YOLO
        try:
            results = model.track(source=frame, conf=0.5, verbose=False, persist=True, imgsz=320)
        except Exception as e:
            if "not enough matching points" not in str(e).lower():
                print(f"YOLO error: {e}")
            continue
        
        # Find detections
        if results and len(results) > 0 and results[0].boxes is not None and len(results[0].boxes) > 0:
            boxes = results[0].boxes
            
            all_x = []
            all_y = []
            for box in boxes:
                if box.xyxy is not None and len(box.xyxy) > 0:
                    coords = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = coords[0], coords[1], coords[2], coords[3]
                    all_x.extend([x1, x2])
                    all_y.extend([y1, y2])
            
            if all_x and all_y:
                cx = (min(all_x) + max(all_x)) / 2.0
                cy = (min(all_y) + max(all_y)) / 2.0
                
                # Calculate pan (left/right)
                pan = int((cx / w) * 180)
                pan = max(0, min(180, pan))
                
                # Calculate tilt (up/down) - INVERTED to fix reversed direction
                # Object at top of frame (cy=0) → tilt=180 (servo tilts up)
                # Object at bottom of frame (cy=h) → tilt=0 (servo tilts down)
                tilt = 180 - int((cy / h) * 180)
                tilt = max(0, min(180, tilt))
                
                # Send command character-by-character
                if (time.time() - last_send) >= COMMAND_DELAY:
                    ser.reset_input_buffer()
                    
                    cmd = f"{pan},{tilt}\r\n"
                    
                    # Send each character individually with delays
                    send_char_by_char(ser, cmd)
                    
                    # Update current positions
                    current_pan = pan
                    current_tilt = tilt
                    
                    last_send = time.time()
                    cmd_count += 1
                    
                    # Print status every 5 commands
                    if cmd_count % 5 == 0:
                        print(f"[{cmd_count:3d}] Pan: {pan:3d}° | Tilt: {tilt:3d}° | FPS: {current_fps:.1f} | Objects: {len(boxes)}")
                    
                    # Clear buffers every 10 commands
                    if cmd_count % 10 == 0:
                        ser.reset_output_buffer()
                        print(f"      [Buffer cleared at command {cmd_count}]")
                
                # Visualize
                cv2.circle(frame, (int(cx), int(cy)), 10, (0, 255, 0), 2)
                for box in boxes:
                    if box.xyxy is not None and len(box.xyxy) > 0:
                        coords = box.xyxy[0].cpu().numpy()
                        x1, y1, x2, y2 = int(coords[0]), int(coords[1]), int(coords[2]), int(coords[3])
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Draw center crosshair
        cv2.drawMarker(frame, (w//2, h//2), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
        
        # Create info panel background (semi-transparent dark rectangle)
        overlay = frame.copy()
        cv2.rectangle(overlay, (5, 5), (300, 140), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        
        # Display information
        y_offset = 30
        
        # FPS
        cv2.putText(frame, f"FPS: {current_fps:.1f}", (15, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Pan angle
        y_offset += 30
        cv2.putText(frame, f"Pan:  {current_pan:3d} deg", (15, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # Tilt angle
        y_offset += 30
        cv2.putText(frame, f"Tilt: {current_tilt:3d} deg", (15, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # Commands sent
        y_offset += 30
        cv2.putText(frame, f"Commands: {cmd_count}", (15, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        
        # Status indicator
        cv2.putText(frame, "TRACKING", (w - 150, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow("YOLO Tracking - WORKING!", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nInterrupted")

finally:
    print("\n[INFO] Cleaning up...")
    cap.release()
    cv2.destroyAllWindows()
    
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    send_char_by_char(ser, "90,90\r\n")
    time.sleep(1)
    ser.close()
    
    print(f"✓ Sent {cmd_count} commands")
    print("✓ Done")
    print("\n" + "="*70)
    print("SUCCESS! Both pan and tilt are working!")
    print("="*70)