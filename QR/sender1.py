import cv2, socket, struct, time
#here pi will send the camera feed to base station by capturing frame by frame
BASE_IP = '169.254.57.35'  
VIDEO_PORT = 8000
RETRY_DELAY = 2  #if connection fail kisi bhi condition m toh uske liye retry krne se phle kitni der rukhna hai 
def connect():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((BASE_IP, VIDEO_PORT))
            print("[cam_sender] connected to base")
            return s
        except Exception as e: #if there comes an error in connection
            print("retrying...", e)
            time.sleep(RETRY_DELAY)

def main():
    cap = cv2.VideoCapture(0)   
    if not cap.isOpened():
        print("camera not found")
        return
    sock = connect()
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            result, encimg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60]) #conversion
            if not result:
                continue
            data = encimg.tobytes()
            try:
                sock.sendall(struct.pack('>I', len(data)) + data)
            except:
                print("connection lost, reconnecting...")
                sock.close()
                sock = connect() #reconnection starts 
    finally:
        cap.release()
        sock.close()
if __name__ == "__main__":
    main()