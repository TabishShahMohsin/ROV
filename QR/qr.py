import cv2
import numpy as np
from scipy.spatial import distance as dist
import math
import logging
import time # For timestamps and simulation
from collections import Counter # For majority vote

# --- 1. SET UP LOGGING ---
# This is perfect. Logs to 'mission_log.txt'.
logging.basicConfig(
    filename='mission_log.txt', 
    level=logging.INFO, 
    format='%(asctime)s - %(levelname)s - %(message)s'
)
print("Logging to 'mission_log.txt'.")
logging.info("--- New ROV Mission Started ---")

# --- 2. MISSION CONFIGURATION ---
# *** TUNE THESE VALUES ***
# Define the *exact* text you put on your QR codes
VALID_PHASE_1_COMMANDS = ["go left", "go right", "go straight"] 
VALID_PHASE_2_COMMANDS = ["drop left", "drop right"]

# --- 3. ROBUST PRE-PROCESSING (Your Function) ---
def preprocess_for_qr(frame):
    """Applies image enhancements to make QR codes more detectable in bad lighting."""
    try:
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
        # This is great for murky water
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced_gray = clahe.apply(gray)
        
        return enhanced_gray
    except cv2.error as e:
        logging.error(f"cv2 error in preprocess: {e}")
        # Return a simple grayscale if enhancement fails
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# --- 4. Initialization ---
detector = cv2.QRCodeDetector()

try:
    cap = cv2.VideoCapture(0) # Or 1, or gstreamer pipeline
    if not cap.isOpened():
        raise IOError("Cannot open camera.")
    cap.set(3, 640)
    cap.set(4, 480)
    print("Camera feed started.")
    logging.info("Camera feed started.")
except IOError as e:
    print(f"CAMERA ERROR: {e}")
    logging.critical(f"CAMERA ERROR: {e}")
    exit()

# --- Object Tracker Variables (Your Logic) ---
active_trackers = {} # Stores: (center, data, points, frames_unseen)
processed_ids = set()    
next_object_id = 1      
MAX_DISTANCE = 70 # Increased for more erratic ROV movement
MAX_FRAMES_UNSEEN = 20 # Lowered slightly to forget lost QRs faster

# --- State Machine Variables ---
current_state = "STATE_PHASE_1_SCANNING"
phase_1_votes = []
phase_1_decision = None
phase_2_decision = None

print("Starting robust mission loop. Press 'q' to quit...")

# --- 5. Main Mission Loop ---
while True:
    try:
        success, frame = cap.read()
        if not success or frame is None:
            logging.warning("Failed to capture frame.")
            time.sleep(0.1) # Don't spam errors
            continue
        
        # --- 5a. Apply Pre-processing ---
        processed_frame = preprocess_for_qr(frame.copy())

        # --- 5b. Detection (Your Logic) ---
        ret_qr, decoded_info, points, _ = detector.detectAndDecodeMulti(processed_frame)
        
        # --- 5c. Tracking Logic (Your Logic) ---
        current_centers = []
        current_data = []
        current_points = []
        
        if ret_qr:
            for i in range(len(points)):
                # Only track QRs that have data we can decode
                if decoded_info[i]:
                    pts_float = points[i] 
                    pts = pts_float.astype(int)
                    cx = int(np.mean(pts[:, 0]))
                    cy = int(np.mean(pts[:, 1]))
                    current_centers.append((cx, cy))
                    current_data.append(decoded_info[i])
                    current_points.append(pts)

        # --- Tracker Update ---
        if len(active_trackers) == 0:
            # No active trackers, register all new detections
            for i in range(len(current_centers)):
                new_id = f"QR-{next_object_id}"
                active_trackers[new_id] = (current_centers[i], current_data[i], current_points[i], 0)
                next_object_id += 1
        else:
            # Match existing trackers to new detections
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
                # No current detections, increment unseen for all trackers
                for tracker_id, (center, data, pts, unseen) in active_trackers.items():
                    unseen += 1
                    if unseen > MAX_FRAMES_UNSEEN:
                        disappeared_trackers.append(tracker_id)
                    else:
                        active_trackers[tracker_id] = (center, data, pts, unseen)
            
            for tracker_id in disappeared_trackers:
                # print(f"Tracker {tracker_id} lost.") # Too noisy for log
                del active_trackers[tracker_id]
                
        # --- 5d. STATE MACHINE & MISSION LOGIC ---
        # Get list of trackers that are newly identified
        objects_to_process = []
        for (tracker_id, (center, data, pts, unseen)) in active_trackers.items():
            if tracker_id not in processed_ids:
                objects_to_process.append((tracker_id, data))
        
        # --- STATE 1: SCANNING FOR 5 QRs ---
        if current_state == "STATE_PHASE_1_SCANNING":
            for (new_id, new_data) in objects_to_process:
                
                # Check if it's a valid Phase 1 command
                if new_data in VALID_PHASE_1_COMMANDS:
                    processed_ids.add(new_id) # Mark as processed
                    phase_1_votes.append(new_data)
                    
                    log_msg = f"PHASE 1 VOTE ({len(phase_1_votes)}/5) - ID: {new_id}, Data: {new_data}"
                    print(log_msg)
                    logging.info(log_msg)
                
                else:
                    # It's a QR, but not one we care about (maybe it's Phase 2?)
                    log_msg = f"PHASE 1 IGNORED QR - ID: {new_id}, Data: {new_data}"
                    print(log_msg)
                    logging.warning(log_msg)

            # --- Check for Phase 1 Completion ---
            if len(phase_1_votes) == 5:
                # Use Counter to find the majority
                vote_counts = Counter(phase_1_votes)
                phase_1_decision = vote_counts.most_common(1)[0][0]
                
                log_msg = f"--- PHASE 1 COMPLETE ---"
                print(log_msg); logging.info(log_msg)
                
                log_msg = f"All Votes: {phase_1_votes}"
                print(log_msg); logging.info(log_msg)
                
                log_msg = f"Decision: GO {phase_1_decision}"
                print(log_msg); logging.info(log_msg)
                
                # --- ROV COMMAND ---
                print(f"*** ROV_CMD: NAVIGATE {phase_1_decision} ***")
                # rov.navigate_to(phase_1_decision) # Your real code here
                # rov.wait_for_arrival()            # Your real code here
                
                current_state = "STATE_PHASE_2_SCANNING"
                
                # *** CRITICAL STEP ***
                # Clear all trackers so we can find the NEW Phase 2 QR code
                print("Clearing trackers, waiting for arrival at Phase 2...")
                logging.info("Clearing trackers for Phase 2.")
                active_trackers.clear()
                processed_ids.clear()
                next_object_id = 1
                
                # Simulate travel time. Replace this with a real check.
                # time.sleep(5.0) 
                print("Arrived at Phase 2. Now scanning for DROP command.")
        
        # --- STATE 2: SCANNING FOR DROP QR ---
        elif current_state == "STATE_PHASE_2_SCANNING":
            if objects_to_process:
                # We only care about the *first* one we see
                (new_id, new_data) = objects_to_process[0]
                
                if new_data in VALID_PHASE_2_COMMANDS:
                    processed_ids.add(new_id) # Mark as processed
                    phase_2_decision = new_data
                    
                    log_msg = f"--- PHASE 2 COMPLETE ---"
                    print(log_msg); logging.info(log_msg)
                    
                    log_msg = f"Drop Instruction: {phase_2_decision}"
                    print(log_msg); logging.info(log_msg)
                    
                    # --- ROV COMMANDS ---
                    print(f"*** ROV_CMD: PICKUP_WEIGHT ***")
                    # rov.pickup_weight() # Your real code here
                    # time.sleep(3.0) # Simulate pickup
                    
                    print(f"*** ROV_CMD: NAVIGATE {phase_2_decision} ***")
                    # rov.navigate_to(phase_2_decision) # Your real code here
                    # time.sleep(5.0) # Simulate travel
                    
                    print(f"*** ROV_CMD: DROP_WEIGHT ***")
                    # rov.drop_weight() # Your real code here
                    
                    current_state = "STATE_MISSION_COMPLETE"
                    log_msg = "--- MISSION COMPLETE ---"
                    print(log_msg); logging.info(log_msg)
                    
                else:
                    log_msg = f"PHASE 2 IGNORED QR - ID: {new_id}, Data: {new_data}"
                    print(log_msg)
                    logging.warning(log_msg)

        # --- STATE 3: MISSION DONE ---
        elif current_state == "STATE_MISSION_COMPLETE":
            # Do nothing, just idle.
            # You could have the ROV surface here.
            pass

        # --- 5e. DRAWING AND DISPLAY (HUD) ---
        
        # Draw trackers
        for (tracker_id, (center, data, pts, unseen)) in active_trackers.items():
            color = (0, 0, 255) # Red = Ignored/Old
            if tracker_id in processed_ids:
                color = (0, 255, 0) # Green = "Processed"
            elif tracker_id in [obj[0] for obj in objects_to_process]:
                color = (255, 255, 255) # White = "New (this frame)"
                
            cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=2)
            text_anchor = (pts[0][0], pts[0][1] - 10)
            cv2.putText(frame, f"{tracker_id}: {data}", text_anchor, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Draw State HUD
        hud_text = f"STATE: {current_state}"
        hud_color = (0, 255, 255) # Yellow
        if current_state == "STATE_MISSION_COMPLETE":
            hud_color = (0, 255, 0) # Green
            
        cv2.putText(frame, hud_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, hud_color, 2)
        
        # Draw Vote Count
        if current_state == "STATE_PHASE_1_SCANNING":
            vote_text = f"Votes: {len(phase_1_votes)}/5"
            cv2.putText(frame, vote_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        elif phase_1_decision:
             vote_text = f"Phase 1: {phase_1_decision}"
             cv2.putText(frame, vote_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
        cv2.imshow("ROV Mission Control (Press 'q' to quit)", frame)
        
        # Optional: Display the pre-processed frame for debugging
        # cv2.imshow("Pre-Processed View", processed_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            logging.info("User pressed 'q' to quit.")
            break

    except Exception as e:
        # This is a critical catch-all for the competition.
        # It logs the error and tries to continue.
        print(f"!!! MAIN LOOP ERROR: {e} !!!")
        logging.error(f"MAIN LOOP CRASH: {e}", exc_info=True)
        time.sleep(0.5) # Prevent 100% CPU on repeated fast failure

# --- 6. Cleanup ---
logging.info("--- Mission Stopped ---")
print("Stopping camera feed. Log file saved.")
cap.release()
cv2.destroyAllWindows()