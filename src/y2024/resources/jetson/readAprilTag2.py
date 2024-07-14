from pupil_apriltags import Detector
import numpy as np
import time
import cv2
import threading
from networktables import NetworkTables

NetworkTables.initialize(server='roborio-846-frc.local')
table = NetworkTables.getTable("AprilTags")

at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

cameraMatrix = np.array([(336.7755634193813, 0.0, 333.3575643300718), (0.0, 336.02729840829176, 212.77376312080065), (0.0, 0.0, 1.0)])

camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )

queue = []

def process_frames():
    global table
    global queue

    camLatency = 0.01

    validTagIds = [4, 7, 3, 8]

    CAM_FOV_H = 63.1
    CAM_FOV_V = 50.0

    CAM_SZ_H = 640
    CAM_SZ_V = 480

    while True:
        if len(queue) == 0: continue

        frame = queue.pop(0)

        process_start_time = time.time()

        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tags = at_detector.detect(img, False, camera_params, 0.065)

        table.putNumber("tx", 0.0)
        table.putNumber("ty", 0.0)
        table.putNumber("latency", 0.0)
        table.putNumber("tid", -1)

      
        try:
            if tags == []: continue

            tag = tags.__getitem__(0)

            if not (tag.tag_id in validTagIds): continue

            cx, cy = tag.center

            tx = (cx - CAM_SZ_H/2) * (CAM_FOV_H / CAM_SZ_H)
            ty = -((cy - CAM_SZ_V/2) * (CAM_FOV_V / CAM_SZ_V))

            table.putNumber("tid", tag.tag_id)
            table.putNumber("tx", tx)
            table.putNumber("ty", ty)
            table.putNumber("latency", camLatency + time.time() - process_start_time)

        except:
            pass

        NetworkTables.flush()
                


if __name__ == "__main__":

    print("SCRIPT: ATTEMPTING CAM 1")
    cap = cv2.VideoCapture("/dev/video0")
    if cap.isOpened() == False:
        print("SCRIPT: ATTEMPTING CAM 2")
        cap = cv2.VideoCapture("/dev/video1")
    if cap.isOpened() == False:
        print("SCRIPT: CAM CONN FAILURE")
    else:
        print("SCRIPT: CAM CONN")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

    if cap.isOpened():
        t1 = threading.Thread(target=process_frames)
        t1.start()
        
        while True:
            _, frame = cap.read()

            if frame is None:
                continue

            queue.append(frame)

            if (len(queue) > 5):
                queue.pop(0)

    cap.release()