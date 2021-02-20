#For Autonomous vision and ball detection
import cv2
import time
import numpy as np
import math
import cscore
from networktables import NetworkTables
import logging
import nt
import wpilib
import wpilib.drive
import ctre



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
camHgt = 480 #352 

#variables to perform distance calculations
DefaultImageHeight = 240 
DefaultBallRadiusInch = 3.50
CalibrationDistanceInch = 16.0
DefaultCameraViewangle = 36 # have to test and modify
HeightOfCamera = 18
#Don't know the above value yet, so have to as the build team
CameraMountingAngleRadians = 0.5 ###(Have to calculate in radians so remember to np.pi/180) Placeholder below
###
MaxPossibleAngle = 60 #Degrees
MaxPossibleDistance = 120.0 #Inches

#Setting up variables for distance calculations
MaxPossibleRadius = (DefaultImageHeight / 2) #MEASURED IN INCHES 
MinPossibleRadius = 0   #WE NEED TO CALCULATE THIS
DefaultPixelsPerInch = (DefaultImageHeight/2)/(math.tan(float(DefaultCameraViewAngle /2) * (np.pi/180)) * CalibrationDistanceInch)
DefaultBallHeightPixel = DefaultBallRadiusInch/(math.tan(float(DefaultCameraViewAngle /2) * (np.pi/180)) * CalibrationDistanceInch) * DefaultImageHeight

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

class MyRobot(wpilib.TimedRobot):


    def robotInit(self):
        gyroChannel = 0  # analog input

        self.left = wpilib.Spark(0)
        self.right = wpilib.Spark(1)

        self.gyro = ctre.PigeonIMU(gyroChannel)
    
        self.myRobot = wpilib.drive.DifferentialDrive(self.left, self.right)
        self.myRobot.setExpiration(0.1)
    
        self.stick = wpilib.XboxController(0)

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

    #If getting camera feed from network table is an issue then switch to front camera
    try:
        # Reset the amount of times the program has not gotten the ball
        getNewBall = 0
        #Reset the thing that tells the code to calculate stuff
        runCalculation = True
        SmartDashboard.putNumber("VisionCounter", VisionCounter)

    except:
        getNewBall = 0
        runCalculation = True
        print("Couldn't get cameraFeed value because no network table was found \n Default to 0")

    #If network tables is causing issue then report it
    try:
        SmartDashboard.putString("VisionCodeSelected", "0")
    except:
        print("Cannot put string because network table was not found")
        logging.warning("Tables not found")

    #Creating an opencv sink 
    cvSink = cscore.CvSink("cvsink")
    cvSink.setSource(Front)
                
    #taking an image and storing it in the img variable
    time0, img = cvSink.grabFrame(img)

    img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

    #Convert the RGB image to HSV since we are using HSV to detect the ball
    imHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #Pixels that represent the ball are made into a white color, whereas everything else becomes black                                      
    InRange = cv2.inRange(imHSV, minHSVBall, maxHSVBall)

    #Blurring so sharp pixel edges get curved, making it easier for the robot to detect
    InRange = cv2.GaussianBlur(InRange, (5, 5), cv2.BORDER_DEFAULT)

    # uses InRange values and these parameters to determine what the ball is and what isn't
    circles = cv2.HoughCircles(InRange, cv2.HOUGH_GRADIENT, 1, int(width/10), param1=180, param2=10, minRadius=5, maxRadius=50)
   
    try:
         #Convert the array to unsigned 16 integers
        circles = np.uint16(np.around(circles))
    except AttributeError:
        #If nothing in array then:
        runCalculation = False
        repeatPolyFit = 0
        logging.warning("No ball was found at this time")

    
    if runCalculation:
        global biggest_radius
        global biggestX
        global biggestY
        global startTime
        global ZDistance
        global DirectDistanceBallInch
        biggest_radius = 0
        for i in circles[0,:]:
            try:

                if (biggest_radius < i[2]):
                    biggest_radius = i[2]
                    biggestX = i[0]
                    biggestY = i[1]

                if saveImage:
                    if(imageCounter % 150 == 0):
                        # Mark circumference of ball on image
                        cv2.circle(img,(biggestX,biggestY),biggest_radius,(0,255,0),2)
                        # mark center of circle
                        cv2.circle(img, (biggestX,biggestY),2,(0,0,255),3)

            except IndexError:
                #If no ball in array
                print("No ball found")
                logging.warning("No values in the hough circle array")
         # Flag for saving images
        if saveImage:
            if(imageCounter % 150 == 0):
                filename = "Image%d-%d.jpg" % (MatchNumber, imageCounter)
                cv2.imwrite(filename, img)
                msgFilewrite = "Look at %s" % filename
                logging.info(msgFilewrite)
            
        #Calculations to determine distance from the balls
        #This will be the backup plan to determine which path we will use if just the imaging does not work
        ActualBallHeightPixel = DefaultBallHeightPixel / (DefaultImageHeight / height)
        ActualPixelsPerInch = DefaultPixelsPerInch / (DefaultImageHeight / height)

        DirectDistanceBallInch = ((ActualBallHeightPixel / (2 * biggest_radius)) * CalibrationDistanceInch)
        XDisaplacementPixel = biggestX - (width / 2)
        YDisplacmentPixel = biggestY - (height / 2)
        YAngle = math.atan(YDisplacmentPixel/(ActualPixelsPerInch * DefaultPixelsPerInch)) #MEASURED IN RADIANS

        XAngle = math.atan(XDisaplacementPixel/(ActualPixelsPerInch * DefaultPixelsPerInch)) * (180/np.pi) #MEASURED IN DEGREES

        ZDistance = DirectDistanceBallInch * math.cos((CameraMountingAngleRadians - YAngle))
        

        #Calculate the relative end time which is NOW
        relativeEndTime = time.time()
        #Subtract to find how many seconds have passed since we found the start time
        timeLapsed = relativeEndTime - startTime
        #Automatically set that we are not ready for any prediction
        readyForPrediction = False

         #run the following code if we have missed less than 3 balls
        if(getNewBall < 2):
            #Fill the array 6 times for each x and y value of the 3 balls in the path
            if(repeatPolyFit < 4):
                #Add distance and angle values to the smoothening class
                SmoothDistance.AddValue("Y", repeatPolyFit, ZDistance)
                SmoothDistance.AddValue("X", repeatPolyFit, timeLapsed)
                SmoothAngle.AddValue("Y", repeatPolyFit, XAngle)
                SmoothAngle.AddValue("X", repeatPolyFit, timeLapsed)
                #Increment since we have a value now
                repeatPolyFit += 1
            else:
                #If we have filled the array, then we are ready for prediction
                readyForPrediction = True
                print("We are ready to predict where the ball is")

            if(readyForPrediction):
                #Get a prediction from the smoothen class
                answer = SmoothDistance.ReturnPrediction()
                answerAngle = SmoothAngle.ReturnPrediction()
                print("ZDistance = " + str(ZDistance))
                msgRawDistance = "RawDistance: %d" % ZDistance
                logging.info(msgRawDistance)
                print("XAngle = " + str(XAngle))
                msgRawAngle = "RawAngle: %d" % XAngle
                logging.info(msgRawAngle)
                print("PredtionDistance = " + str(answer))
                print("PredictionAngle = " + str(answerAngle))

                #Determining the X, Y and Z distances and/or angles in order to test against our predictons
                if (abs(ZDistance - answer) < 6.56 ):
                    SmoothDistance.AppendValues("Y", ZDistance)
                    ZDistance = answer
                    endTime = time.time()
                    elapsedTime = endTime - startTime
                    print("FeedingTime = " + str(elapsedTime))
                    SmoothDistance.AppendValues("X", elapsedTime)
                elif (abs(ZDistance - answer) > 6.56 ):
                    getNewBall += 1
                    answer = 0
                    SmoothDistance.AppendValues("Y", ZDistance)
                    endTime = time.time()
                    elapsedTime = endTime - startTime
                    SmoothDistance.AppendValues("X", elapsedTime)
                elif (abs(XAngle - answerAngle) < 2.0):
                    SmoothAngle.AppendValues("Y", XAngle)
                    XAngle = answerAngle
                    endTime = time.time()
                    elapsedTime = endTime - startTime
                    SmoothAngle.AppendValues("X", elapsedTime)


            elif (abs(ZDistance - answer) > 2.0 ):
                getNewBall += 1
                answer = 0
                SmoothAngle.AppendValues("Y", XAngle)
                endTime = time.time()
                elapsedTime = endTime - startTime
                SmoothAngle.AppendValues("X", elapsedTime)
        elif (getNewBall == 4):
            repeatPolyFit = 0
            getNewBall = 0
            ZDistance = 0
            XAngle = 0
            startTime = time.time()
                
            SmartDashboard.putNumber("ZDistance", ZDistance)
            msgDistance = "Predicted distance: %d" % ZDistance
            logging.info(msgDistance)
            SmartDashboard.putNumber("XAngle", XAngle)
            msgAngle = "XAngle: %d" % XAngle
            logging.info(msgAngle)

        else:
            print("could not find any object so decided to skip calculations as well")
            ZDistance = 0
            XAngle = 0
            SmartDashboard.putNumber("ZDistance", ZDistance)
            print("ZDistance = " + str(ZDistance))
            SmartDashboard.putNumber("XAngle", XAngle)
            print("XAngle = " + str(XAngle))
                
        print("The time it took " + str(time.time()-t0))


while True:
    time.sleep(2)
    Vision()

"""
        EXTREMELY OVERSIMPLIFIED psuedocode

        if ZDistance>= some value
            run the red path(hardcoding function with instructions for robot, i.e direction to move in, turning degrees, etc.)

        else 
            run blue path
"""


# What the code would look like (Unifinished as of now)
"""   
def BluePath():
    print("Bluepath")
    
def RedPath():
    print("Redpath")


if DirectDistanceBallInch>=180:#inches
    BluePath()
else:
    RedPath()

"""