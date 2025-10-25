import cv2
import numpy as np
from scipy.spatial import distance as dist
import math
import logging
import time # For timestamps

# --- 1. SET UP LOGGING ---
# This replaces all network code.
# It will create a 'mission_log.txt' file in the same directory.
logging.basicConfig(
    filename='mission_log.txt', 
    level=logging.INFO, 
    format='%(asctime)s - %(message)s'
)
print("Logging QR data to 'mission_log.txt'.")
logging.info("--- New ROV Mission Started ---")

# --- 2. PRE-PROCESSING FUNCTION ---
def preprocess_for_qr(frame):
    """Applies image enhancements to make QR codes more detectable in bad lighting."""
    try:
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced_gray = clahe.apply(gray)
        
        # OpenCV's detector often works better on the enhanced grayscale image
        # directly, rather than a hard binary threshold.
        return enhanced_gray
    except cv2.error:
        # If frame is bad, return the original
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


# --- Initialization ---
detector = cv2.QRCodeDetector()

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# --- Object Tracker Variables ---
active_trackers = {} # Stores: (center, data, points, frames_unseen)
processed_ids = set()    
next_object_id = 1       
MAX_DISTANCE = 60 # Increased for more erratic ROV movement
MAX_FRAMES_UNSEEN = 30 # Increased for stability

print("Starting robust camera feed. Press 'q' to quit...")

# --- Main Loop ---
while True:
    success, frame = cap.read()
    if not success:
        print("Failed to capture frame.")
        break
    
    # --- 3. APPLY PRE-PROCESSING ---
    # We will run the detector on the enhanced frame
    processed_frame = preprocess_for_qr(frame)

    # --- Detection and Counting ---
    ret_qr, decoded_info, points, _ = detector.detectAndDecodeMulti(processed_frame)
    
    visible_qr_count = 0
    if ret_qr:
        visible_qr_count = len(points) 

    count_text = f"Visible QRs: {visible_qr_count}"
    # Draw count on the original color frame
    cv2.putText(frame, count_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                0.7, (0, 255, 255), 2) 

    # --- Tracking Logic ---
    current_centers = []
    current_data = []
    current_points = []
    
    if ret_qr:
        for i in range(len(points)):
            if decoded_info[i]:
                # Use the 'points' array from the detection
                pts_float = points[i] 
                pts = pts_float.astype(int)
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
                current_centers.append((cx, cy))
                current_data.append(decoded_info[i])
                current_points.append(pts)

    if len(active_trackers) == 0:
        for i in range(len(current_centers)):
            new_id = f"QR-{next_object_id}"
            active_trackers[new_id] = (current_centers[i], current_data[i], current_points[i], 0)
            next_object_id += 1
            
    else:
        tracker_ids = list(active_trackers.keys())
        tracker_info = list(active_trackers.values())
        tracker_centers = [info[0] for info in tracker_info]
        disappeared_trackers = []

        if len(current_centers) > 0:
            D = dist.cdist(np.array(tracker_centers), np.array(current_centers))
            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]
            
            used_rows = set()
            used_cols = set()
            
            for (row, col) in zip(rows, cols):
                if row in used_rows or col in used_cols:
                    continue
                if D[row, col] > MAX_DISTANCE:
                    continue
                
                tracker_id = tracker_ids[row]
                active_trackers[tracker_id] = (current_centers[col], current_data[col], current_points[col], 0)
                used_rows.add(row)
                used_cols.add(col)
            
            unused_rows = set(range(0, D.shape[0])).difference(used_rows)
            unused_cols = set(range(0, D.shape[1])).difference(used_cols)

            for row in unused_rows:
                tracker_id = tracker_ids[row]
                (center, data, pts, unseen) = active_trackers[tracker_id]
                unseen += 1
                if unseen > MAX_FRAMES_UNSEEN:
                    disappeared_trackers.append(tracker_id)
                else:
                    active_trackers[tracker_id] = (center, data, pts, unseen)

            for col in unused_cols:
                new_id = f"QR-{next_object_id}"
                active_trackers[new_id] = (current_centers[col], current_data[col], current_points[col], 0)
                next_object_id += 1
        else:
            for tracker_id, (center, data, pts, unseen) in active_trackers.items():
                unseen += 1
                if unseen > MAX_FRAMES_UNSEEN:
                    disappeared_trackers.append(tracker_id)
                else:
                    active_trackers[tracker_id] = (center, data, pts, unseen)
        
        for tracker_id in disappeared_trackers:
            print(f"Tracker {tracker_id} lost.")
            del active_trackers[tracker_id]
            
    # --- Drawing and Processing Loop ---
    objects_to_process = []
    
    for (tracker_id, (center, data, pts, unseen)) in active_trackers.items():
        color = (0, 255, 0) # Green = "Processed"
        
        if tracker_id not in processed_ids:
            color = (255, 255, 255) # White = "New"
            objects_to_process.append((tracker_id, data))
            
        # Draw on the original color frame
        cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=3)
        text_anchor = (pts[0][0], pts[0][1] - 10)
        cv2.putText(frame, tracker_id, text_anchor, cv2.FONT_HERSHEY_SIMPLEX, 
                    0.7, color, 2)

    # --- Display the original color frame ---
    cv2.imshow("ROV QR Tracker (Press 'q' to quit)", frame)
    
    # Optional: Display the pre-processed frame for debugging
    # cv2.imshow("Pre-Processed View", processed_frame)

    # --- 4. RUN LOGGING after drawing ---
    for (new_id, new_data) in objects_to_process:
        processed_ids.add(new_id)
        
        print(f"\n--- NEW QR OBJECT: {new_id} ---")
        print(f"Data: {new_data}")
        
        # Log the data to the file
        logging.info(f"ID: {new_id}, Data: {new_data}")
        
        # You can add ROV commands here based on the data, e.g.:
        # if new_data == "CMD_STATION_A":
        #    rov.hold_position()
        
        print("------------------------------")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
logging.info("--- Mission Stopped ---")
print("Stopping camera feed. Log file saved.")
cap.release()
cv2.destroyAllWindows()