# import cscore
# import cv2
import time
import numpy as np
# import math

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
English = 0 
