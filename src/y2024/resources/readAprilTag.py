from pupil_apriltags import Detector
from datetime import datetime
import numpy as np
import math
import os
from cv2 import *
import threading
from networktables import NetworkTables

NetworkTables.initialize(server='roborio-846-frc.local')
table = NetworkTables.getTable("AprilTags")

validTagIds = [4, 7, 3, 8]

aprilTagX=[0-14/12.0]*20
aprilTagY=[6-6/12.0]*20

aprilTagX[4]=104.4/12-5.5/12.0 #RED?
aprilTagY[4]=4.4/12+12/12.0

aprilTagX[3]=126.65/12-5.5/12.0 #RED?
aprilTagY[3]=4.4/12+12/12.0

aprilTagX[7]=-104.4/12-5.5/12.0 #BLUE?
aprilTagY[7]=635.8/12+12/12.0

aprilTagX[8]=-126.65/12-5.5/12.0 #BLUE?
aprilTagY[8]=635.8/12+12/12.0

# aprilTagX[16]=0.3-14/12.0
# aprilTagY[16]=9.0247+6/12.0
aprilAngle=[180]*20

locations=np.array([0.0, 0.0, 0.0])

framesSeen=0
confidence=0

lower = np.array([0, 0, 0])
upper = np.array([255, 255, 105])

visualization = False
try:
    import cv2
except:
    raise Exception('opencv not found')

try:
    from cv2 import imshow
except:
    print("The function imshow was not implemented in this installation. Rebuild OpenCV from source to use it")
    print("Visualization will be disabled.")
    visualization = False


def getDistanceAssumingHeadOn(y, verticalPixelHeight):
    heightOfObject =1
    verticalFOV = 1 * 3.14159/360
    return (heightOfObject*verticalPixelHeight)/(2*y*math.tan(verticalFOV/2))

# def getDist(xReal, yReal, verticalPixelHeight, horizontalPixelWidth, tagSize):
#     x = xReal-horizontalPixelWidth/2
#     y=verticalPixelHeight/2-yReal
    

def getDistance(xReal, yReal, verticalPixelHeight, horizontalPixelWidth, tagHeight):
    x = xReal-horizontalPixelWidth/2
    y=verticalPixelHeight/2 -yReal

    diagonalFOV=(68.5)*(3.14159/180)
    f = math.sqrt(horizontalPixelWidth*horizontalPixelWidth+verticalPixelHeight*verticalPixelHeight)/(2*(math.tan(diagonalFOV/2)))
    # 587.4786864517579
    mountHeight=13
    mountAngle=(35)*(3.14159/180)

    VertAngle = mountAngle+math.atan(y/f)
    yDist = (tagHeight-mountHeight)/math.tan(VertAngle)
    xDist = ((tagHeight-mountHeight)/math.sin(VertAngle))*x/(math.sqrt(f*f+y*y))


    return (xDist, yDist, tagHeight)


at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

cameraMatrix = np.array([(336.7755634193813, 0.0, 333.3575643300718), (0.0, 336.02729840829176, 212.77376312080065), (0.0, 0.0, 1.0)])

camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )

starttime = datetime.now()
RealStart = datetime.now()
framesPassed=0
roboRioFrameRequests=0
def run_april (frame):
    global framesPassed
    global starttime
    global roboRioFrameRequests
    global RealStart
    global framesSeen
    global confidence
    global locations
    data = table.getNumberArray("roboRioFrameRequest", [0, 0])
    currentRoboRioFrameRequest=data[0]
    currentBearing=data[1]
    # print(currentRoboRioFrameRequest)
    frameRequested = (currentRoboRioFrameRequest != roboRioFrameRequests) and (currentRoboRioFrameRequest != roboRioFrameRequests -1)
    # print(str(currentRoboRioFrameRequest) +"    " + str(roboRioFrameRequests))


    if (frameRequested):
        sizingFactor = 1/2
        img = frame
        img = cv2.resize(img, ((int)(frame.shape[1]*sizingFactor), (int)(frame.shape[0]*sizingFactor)))
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        #tags = at_detector.detect(img, True, camera_params, parameters['sample_test']['tag_size'])
        tags = at_detector.detect(img, True, camera_params, 0.065)
        bufferSize =4

        if tags != []:
            try:
                print(tags.__getitem__(0).tag_id)   
                
                (ptUpperL,ptUpperR,ptBottomR,ptBottomL) = tags.__getitem__(0).corners

                ptUpperL=((int)(ptUpperL[0]), (int)(ptUpperL[1]))
                ptUpperR=((int)(ptUpperR[0]), (int)(ptUpperR[1]))
                ptBottomL=((int)(ptBottomL[0]), (int)(ptBottomL[1]))
                ptBottomR=((int)(ptBottomR[0]), (int)(ptBottomR[1]))

                upperRight = frame[int(ptUpperR[1]/sizingFactor-bufferSize):int(ptUpperR[1]/sizingFactor+bufferSize), int(ptUpperR[0]/sizingFactor-bufferSize):int(ptUpperR[0]/sizingFactor+bufferSize)]
                
                upperRightHsv = cv2.cvtColor(upperRight, cv2.COLOR_BGR2HSV)
                upperRightMask = cv2.inRange(upperRightHsv, lower, upper)
                upperRightResult =cv2.bitwise_and(upperRight, upperRight, mask=upperRightMask)
                upperRight_np_result = np.array(upperRightResult)
                upperRightResult = cv2.merge((upperRight_np_result, upperRight_np_result, upperRight_np_result))
                upperRightResult = cv2.Canny(upperRightResult, 60, 200)
                upperRightContours, hierarchy = cv2.findContours(upperRightResult, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                upperRightRect = cv2.boundingRect(upperRightContours[0])
                bottomRightBound =(int(ptUpperR[1]/sizingFactor-bufferSize)+upperRightRect[2], int(ptUpperR[0]/sizingFactor-bufferSize)+upperRightRect[3])

                upperLeft = frame[int(ptUpperL[1]/sizingFactor-bufferSize):int(ptUpperL[1]/sizingFactor+bufferSize), int(ptUpperL[0]/sizingFactor-bufferSize):int(ptUpperL[0]/sizingFactor+bufferSize)]
                upperLeftHsv = cv2.cvtColor(upperLeft, cv2.COLOR_BGR2HSV)
                upperLeftMask = cv2.inRange(upperLeftHsv, lower, upper)
                upperLeftResult = cv2.bitwise_and(upperLeft, upperLeft, mask=upperLeftMask)
                upperLeft_np_result = np.array(upperLeftResult)
                upperLeftResult = cv2.merge((upperLeft_np_result, upperLeft_np_result, upperLeft_np_result))
                upperLeftResult = cv2.Canny(upperLeftResult, 60, 200)

                upperLeftContours, hierarchy = cv2.findContours(upperLeftResult, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                upperLeftRect = cv2.boundingRect(upperLeftContours[0])
                cv2.line(upperLeftResult,(upperLeftRect[0], upperLeftRect[1]),(upperLeftRect[0], upperLeftRect[1]+upperLeftRect[3]), color=(0, 255, 255), thickness=1)   
                cv2.line(upperLeftRect,(upperLeftRect[0], upperLeftRect[1]),(upperLeftRect[0]+upperLeftRect[2], upperLeftRect[1]), color=(0, 255, 255), thickness=1)   
                cv2.line(upperLeftResult,(upperLeftRect[0], upperLeftRect[1]+upperLeftRect[3]),(upperLeftRect[0]+upperLeftRect[2], upperLeftRect[1]+upperLeftRect[3]), color=(0, 255, 255), thickness=1)   
                cv2.line(upperLeftResult,(upperLeftRect[0]+upperLeftRect[2], upperLeftRect[1]+upperLeftRect[3]),(upperLeftRect[0]+upperLeftRect[2], upperLeftRect[1]), color=(0, 255, 255), thickness=1)   

                bottomLeftBound=(int(ptUpperL[1]/sizingFactor-bufferSize)+upperLeftRect[0], int(ptUpperL[0]/sizingFactor-bufferSize)+upperLeftRect[3])

                bottomLeft = frame[int(ptBottomL[1]/sizingFactor-bufferSize):int(ptBottomL[1]/sizingFactor+bufferSize), int(ptBottomL[0]/sizingFactor-bufferSize):int(ptBottomL[0]/sizingFactor+bufferSize)]
                bottomLeftHsv =cv2.cvtColor(bottomLeft, cv2.COLOR_BGR2HSV)
                bottomLeftMask=cv2.inRange(bottomLeftHsv, lower, upper)
                bottomLeftResult =cv2.bitwise_and(bottomLeft, bottomLeft, mask=bottomLeftMask)

                bottomLeft_np_result = np.array(bottomLeftResult)
                bottomLeftResult = cv2.merge((bottomLeft_np_result, bottomLeft_np_result, bottomLeft_np_result))
                bottomLeftResult = cv2.Canny(bottomLeftResult, 60, 200)

                bottomLeftContours, hierarchy = cv2.findContours(bottomLeftResult, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                bottomLeftRect = cv2.boundingRect(bottomLeftContours[0])
                upperLeftBound=(int(ptBottomL[1]/sizingFactor-bufferSize)+bottomLeftRect[0], int(ptBottomL[0]/sizingFactor-bufferSize)+bottomLeftRect[1])

                bottomRight = frame[int(ptBottomR[1]/sizingFactor-bufferSize):int(ptBottomR[1]/sizingFactor+bufferSize), int(ptBottomR[0]/sizingFactor-bufferSize):int(ptBottomR[0]/sizingFactor+bufferSize)]
                bottomRightHsv =cv2.cvtColor(bottomRight, cv2.COLOR_BGR2HSV)
                bottomRightMask=cv2.inRange(bottomRightHsv, lower, upper)
                bottomRightResult =cv2.bitwise_and(bottomRight, bottomRight, mask=bottomRightMask)

                bottomRight_np_result = np.array(bottomRightResult)
                bottomRightResult = cv2.merge((bottomRight_np_result, bottomRight_np_result, bottomRight_np_result))
                bottomRightResult = cv2.Canny(bottomRightResult, 60, 200)

                bottomRightContours, hierarchy = cv2.findContours(bottomRightResult, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                bottomRightRect = cv2.boundingRect(bottomRightContours[0])
                print(bottomRightRect)
                upperRightBound=(int(ptBottomR[1]/sizingFactor-bufferSize)+bottomRightRect[1], int(ptBottomR[0]/sizingFactor-bufferSize)+bottomRightRect[2])
                frame=cv2.circle(frame, (upperLeftBound[1], upperLeftBound[0]), radius=0, color=(0, 255, 255), thickness=-1)
                frame=cv2.circle(frame, (upperRightBound[1], upperRightBound[0]), radius=0, color=(0, 255, 255), thickness=-1)
                frame=cv2.circle(frame, (bottomLeftBound[1], bottomLeftBound[0]), radius=0, color=(0, 255, 255), thickness=-1)
                frame=cv2.circle(frame, (bottomRightBound[1], bottomRightBound[0]), radius=0, color=(0, 255, 255), thickness=-1)

                location=(np.array(getDistance(upperRightBound[1], upperRightBound[0], frame.shape[0], frame.shape[1], 60.75))
                +np.array(getDistance(bottomLeftBound[1], bottomLeftBound[0], frame.shape[0], frame.shape[1], 54.5)+np.array(getDistance(bottomRightBound[1], bottomRightBound[0], frame.shape[0], frame.shape[1], 60.75))+np.array(getDistance(upperLeftBound[1], upperLeftBound[0], frame.shape[0], frame.shape[1], 54.5))))/4
                # )
                locations+=location

                print(location)
                tagId=tags.__getitem__(0).tag_id

                table.putNumber("robotX", location[0]/12.0)
                table.putNumber("robotY", location[1]/12.0)
                table.putNumber("aprilTagID", tagId)
                table.putNumber("aprilTagX", aprilTagX[tagId])
                table.putNumber("aprilTagAngle", aprilAngle[tagId])
                table.putNumber("aprilTagY", aprilTagY[tagId])

                if (tagId in validTagIds):
                    table.putNumber("aprilTagConfidence", confidence)
                else:
                    table.putNumber("aprilTagConfidence", 0.0)

                table.putNumber("processorFrameSent", currentRoboRioFrameRequest)
                roboRioFrameRequests=currentRoboRioFrameRequest

                NetworkTables.flush()

                framesSeen+=1;   

            except:
                table.putNumber("aprilTagConfidence", 0.0)
                table.putNumber("processorFrameSent", currentRoboRioFrameRequest)
                roboRioFrameRequests=currentRoboRioFrameRequest
                print("Uh oh! We encountered an error and did not send data this loop")  

        else:
            table.putNumber("aprilTagConfidence", 0.0)
            table.putNumber("processorFrameSent", currentRoboRioFrameRequest)
            roboRioFrameRequests=currentRoboRioFrameRequest

        time = datetime.now()-RealStart

        framesPassed+=1


        if (framesPassed >= 10):
            endtime=datetime.now()

            locations=np.array([0.0, 0.0, 0.0])
            print('FPS: ' + (str)(framesPassed/(endtime-starttime).total_seconds()))
            starttime=endtime
            confidence =framesSeen/(1.0*framesPassed)
            framesPassed=0
            framesSeen=0



def RUN_APRIL_846():
    print("SCRIPT: ATTEMPTING CAM 1")
    cap = cv2.VideoCapture("/dev/video0")
    if cap.isOpened()==False:
        print("SCRIPT: ATTEMPTING CAM 2")
        cap = cv2.VideoCapture("/dev/video1")
    if cap.isOpened()==False:
        print("SCRIPT: CAM CONN FAILURE")
    else:
        print("SCRIPT: CAM CONN")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

    counter = 0
    otime = datetime.now()
    if cap.isOpened():
        while(True):
            counter+=1

            # frame = cv.imread('testing.jpg')
            _, frame = cap.read()

            t1 = threading.Thread(target=run_april, args=(frame,))
            t1.start()
            t1.join()


            if counter % 10 == 0:
                counter = 0
                print("FPS: " + str((datetime.now()-otime)))
                otime = datetime.now()
            
            
            #if cv.waitKey(1)== ord('q'):
            #    break

        
    cap.release()





RUN_APRIL_846()