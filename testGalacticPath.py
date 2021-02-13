import cv2
import time
import numpy as np
import math
#import cscore
from cscore import CameraServer

time.sleep(5)

cs = cscore.CameraServer.GetInstance()


#Exception block for handling network table initialization
try:
    nt.NetworkTables.initialize(server = "10.12.59.2")
    print("Network table successfully found")
    SmartDashboard = nt.NetworkTables.getTable("SmartDashboard")
except:
    print("No netwrk table found, but still continuing with the rest of the code.")

#Set up smoothen class
class SmoothenClass:
    def __init__(self, order):
        self.order = order
        self.x = np.arrange(8.0)
        self.y = np.arrange(8.0)

    def AddValue(self, string, index, float):
        if string == "Y": #make y
            self.y[index] = float
        if string == "X": # make x
            self.x[index] = float

    def AppendValues(self, string, float):
        if string == "Y":
            self.y = np.delete(self.y, 0)
            self.y = np.append(self.y, float)
        if string == "X":
            self.x = np.delete(self.x, 0)
            self.x = np.append(self.x, float)

#Laying down set up for the front camera
def setupFrontCamera():
    global Front
    Front = cs.startAutomaticCapture(name = "Front Camera", path =  "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0")
    Front.setResolution(640, 480) # 640 x 352
    print("Setting up front camera")
#Starting the set up process
setupFrontCamera()

#Hardcoding camera width and height
camWid = 640
camHgt = 480

#variables to perform distance calculations
DefaultImageHeight = 240 
DefaultBallRadiusInch = 3.50
CalibrationDistanceInch = 16.0
DefaultCameraViewangle = 36
###HeightOfCamera = ?
#Don't know the above value yet, so have to as the build team
###CameraMountingAngleRadians = (Have to calculate in radians so remember to np.pi/180)
###
MaxPossibleAngle = 60 #Degrees
MaxPossibleDistance = 120 #Inches

#Setting up variables for distance calculations
MaxPossibleRadius = (DefaultImageHeight / 2) #MEASURED IN INCHES 
MinPossibleRadius = 0   #WE NEED TO CALCULATE THIS
DefaultPixelsPerInch = (DefaultImageHeight/2)/(math.tan((DefaultCameraViewAngle/2) * (np.pi/180)) * CalibrationDistanceInch)
DefaultBallHeightPixel = DefaultBallRadiusInch/(math.tan((DefaultCameraViewAngle/2) * (np.pi/180)) * CalibrationDistanceInch) * DefaultImageHeight

#Maximum and minimum possible HSV values to detect the ball
minHSVBall = np.array([20, 76, 55])
maxHSVBall = np.array([45, 255, 255])
#minHSVBall = np.array([25, 125, 125]) Backup values
#maxHSVBall = np.array([35, 255, 255]) Backup values

#To determine the x and y center of the image 
imageCenterX = camWid/2
imageCenterY = camHgt/2

#Scaling
img = np.zeros(shape=(640, 480, 3), dtype=np.uint8)
scale_percent = 50 # percent of original size
width = int(img.shape[0] * scale_percent / 100)
height = int(img.shape[1] * scale_percent / 100)
dim = (width, height)

#Whether or not to do calculations
runCalculation = True
#Whether or not to save images
saveImage = True
#Helps in labeling images so that they are not overwritten
imageCounter = 0
#How many time to repeat adding value to x and y lists
repeatPolyFit = 0
#Counter to tell whether vision has stopped working or not
VisionCounter = 0
#Timer for the x-value of plotting direction
startTime = time.time()

#Smoothening class which takes the order as a perimeter - SMOOTHING DISTANCE
SmoothDistance = SmoothenClass(1)
#Smoothening class which takes the order as a perimeter - SMOOTHING ANGLE
SmoothAngle = SmoothenClass(1)

def Vision():
    #Timer used to determine how many seconds it took to run vision helps with FPS
    t0 = time.time()

    #Variables that needed to be global so the function could use them 
    global img
    global imageCounter
    global VisionCounter
    global repeatPolyFit
    global SmoothDistance
    global SmoothAngle
    global runCalculation

    #Increasing by 1 so hat each image will be stored with a different name and they do not overwrite one another
    imageCounter += 1
    #Increment this counter so that changing values represent working vision
    VisionCounter += 1

    #If getting camera feed from network table is an issue then default to using front camera
    try:
        # Reset the amount of times the program has not gotten the ball
        getNewBall = 0
        #Reset the thing that tells the code to calculate stuff
        runCalculation = True
        SmartDashboard.putNumber("VisionCounter", VisionCounter)

    except:
        getNewBall = 0
        runCalculation = True
        print("Couldn't get cameraFeed value because no network table was found\nDefault to 0")

    #If network tables is causing issue then report it
    try:
        SmartDashboard.putString("VisionCodeSelected", "0")
    except:
        print("Cannot put string because network table was not found")
        logging.warning("Network Tables not found")

    #Creating an opencv sink 
    cvSink = cscore.CvSink("cvsink")
    cvSink.setSource(Front)
                
    #taking an image and storing it in the img variable
    time0, img = cvSink.grabFrame(img)

    img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

    #Convert the RGB image to HSV
    imHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #Find pixels that represent the ball and makes it white, everything else becomes black                                         
    InRange = cv2.inRange(imHSV, minHSVBall, maxHSVBall)

    #Blurring so that the sharp edges of the pixels of the rendered ball bcome smoother, hence easier for the robot to detect
    InRange = cv2.GaussianBlur(InRange, (5, 5), cv2.BORDER_DEFAULT)

    # Hough circle transformation looks in InRange file to find circles given the parameters below
    circles = cv2.HoughCircles(InRange, cv2.HOUGH_GRADIENT, 1, int(width/10), param1=180, param2=10, minRadius=5, maxRadius=50)
   
    try:
         #Convert the array to unsigned 16 integers
        circles = np.uint16(np.around(circles))
    except AttributeError:
        #If nothing in array then:
        runCalculation = False
        repeatPolyFit = 0
        logging.warning("No ball was found at this time")












