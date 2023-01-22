#!/usr/bin/env python3
#----------------------------------------------------------------------------------------------#
#                               North Canton Hoover High School                                #
#                                                                                              #
#                                Team 4121 - Norsemen Robotics                                 #
#                                                                                              #
#                               Vision & Motion Processing Code                                #
#----------------------------------------------------------------------------------------------#
#                                                                                              #
#  This code continuously analyzes images from one or more USB cameras to identify on field    #
#  game pieces and vision targets.  For game pieces, the code will identify all game pieces    #
#  within the camera's field of view and determine the closest one.  The distance and angle    #
#  to the closest game piece is calculated and made available to the main robot code through   #
#  network tables.  The closest game piece is highlighted with a green box while all other     #
#  found game pieces are highlighted with a red box.  The annotated video is streamed to the   #
#  driver station for display.  The annotated video is also saved to a file for post game      #
#  review and analysis.  For vision targets, the code will identify all vision targets and     #
#  calculate the angle and distance to each one.  Vision target information is made available  #
#  to the main robot code through network tables.                                              #
#                                                                                              #
#  This code also continuously interrogates a VMX-Pi board to determine linear and angular     #
#  motion in all three axes.  This information is made available to the main robot code        #
#  through network tables.                                                                     #
#                                                                                              #
#----------------------------------------------------------------------------------------------#
#                                                                                              #
#  Authors:  Jonas Muhlenkamp                                                                  #
#            Ricky Park                                                                        #
#            Tresor Nshimiye                                                                   #
#            Tim Fuller - Mentor                                                               #
#                                                                                              #
#  Creation Date: 3/1/2018                                                                     #
#                                                                                              #
#  Revision: 6.0                                                                               #
#                                                                                              #
#  Revision Date: 3/14/2021                                                                    #
#                                                                                              #
#----------------------------------------------------------------------------------------------#

#System imports
import sys

#Setup paths
sys.path.append('/home/pi/.local/lib/python3.7/site-packages')
sys.path.append('/home/pi/Team4121/Libraries')
sys.path.append('/usr/local/lib/vmxpi/')

#sys.path.append('C:\\Users\\timfu\\Documents\\Team4121\\Libraries')

#Module imports
import cv2 as cv
import numpy as np
import datetime
import time
import logging
import argparse
from operator import itemgetter
import math
from networktables import NetworkTables
from time import sleep

#Team 4121 module imports
from FRCVisionLibrary import VisionLibrary
from FRCCameraLibrary import FRCWebCam
from FRCNavxLibrary import FRCNavx

#Set up basic logging
logging.basicConfig(level=logging.DEBUG)

#Declare global variables
cameraFile = '/home/pi/Team4121/Config/2023CameraSettings.txt'
visionFile = '/home/pi/Team4121/Config/2023VisionSettings.txt'
videoDirectory = '/home/pi/Team4121/Videos'
cameraValues={}

#Define program control flags
useNavx = True
videoTesting = True
resizeVideo = False
saveVideo = False
navxTesting = True

#Define main processing function
def main():
    global useNavx
    
    #Define flags
    networkTablesConnected = False

    #Define variables
    gyroAngle = 0
    currentTime = []
    timeString = ''

    #Define objects
    navx = None
    visionTable = None
    navxTable = None

    #Create Navx object
    if useNavx:
        navx = FRCNavx('NavxStream')
        timeString = navx.get_raw_time()
        useNavx = not navx.poisoned
    else:
        currentTime = time.localtime(time.time())
        timeString = str(currentTime.tm_year) + str(currentTime.tm_mon) + str(currentTime.tm_mday) + str(currentTime.tm_hour) + str(currentTime.tm_min)

    #Open a log file
    logFilename = '/home/pi/Team4121/Logs/Run_Log_' + timeString + '.txt'
    log_file = open(logFilename, 'w')
    log_file.write('run started on {}.\n'.format(datetime.datetime.now()))
    log_file.write('')

    #Connect NetworkTables
    try:
        NetworkTables.initialize(server='10.41.21.2')
        visionTable = NetworkTables.getTable("vision")
        navxTable = NetworkTables.getTable("navx")
        networkTablesConnected = True
        log_file.write('Connected to Networktables on 10.41.21.2 \n')

        visionTable.putNumber("RobotStop", 0)
    except:
        log_file.write('Error:  Unable to connect to Network tables.\n')
        log_file.write('Error message: ', sys.exc_info()[0])
        log_file.write('\n')

    print("connected to table" if networkTablesConnected else "printing output")

    #Start main processing loop
    while (True):

        #####################
        # Process NavX Gyro #
        #####################

        #Get VMX gyro angle
        if useNavx:

            gyroInit = navxTable.getNumber("ZeroGyro", 0)  #Check for signal to re-zero gyro
            if gyroInit == 1:
                navx.reset_gyro()
                navxTable.putNumber("ZeroGyro", 0)      
            gyroAngle = navx.read_angle()  #Read gyro angle

        else:
            gyroAngle = -9999  #Set default gyro angle
   
        #Put gyro value in NetworkTables
        if networkTablesConnected:
            navxTable.putNumber("GyroAngle", gyroAngle)
        if navxTesting:
            print(gyroAngle)

        #################################
        # Check for stopping conditions #
        #################################

        #Check for stop code from keyboard (for testing)
        if videoTesting == True:
            if cv.waitKey(1) == 27:
                break

        #Check for stop code from network tables
        if networkTablesConnected == True: 
            robotStop = visionTable.getNumber("RobotStop", 0)
            if (robotStop == 1) or (networkTablesConnected == False):
                break

        #Pause before next analysis
        #time.sleep(0.066) #should give ~15 FPS

    #Close all open windows (for testing)
    if videoTesting == True:
        cv.destroyAllWindows()

    #Close the log file
    log_file.write('Run stopped on {}.'.format(datetime.datetime.now()))
    log_file.close()


#define main function
if __name__ == '__main__':
    main()
