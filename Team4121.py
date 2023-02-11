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
from platform import node as hostname

#Team 4121 module imports
from FRCCameraLibrary import FRCWebCam
from FRCNavxLibrary import FRCNavx
from FRCVision2023 import *

#Set up basic logging
logging.basicConfig(level=logging.DEBUG)

#Declare global variables
cameraFile = '/home/pi/Team4121/Config/2023CameraSettings.txt'
visionFile = '/home/pi/Team4121/Config/2023VisionSettings.txt'
videoDirectory = '/home/pi/Team4121/Videos'
cameraValues = {}

#Define program control flags
useNavx = False
useVision = False
videoTesting = False
resizeVideo = False
saveVideo = False
findField = False
findTape = False
navxTesting = 0 # 0 to disable
visionTesting = 0 # 0 to disable

piname = hostname()
currentTime = time.localtime(time.time())
timeString = str(currentTime.tm_year) + str(currentTime.tm_mon) + str(currentTime.tm_mday) + str(currentTime.tm_hour) + str(currentTime.tm_min)

# change settings based on hostname
if piname == 'raspberrypi3':
    useNavx = True
    useVision = False
    videoTesting = False
    resizeVideo = False
    saveVideo = False
    findTape = True
    navxTesting = 0
    visionTesting = 0
elif piname == 'raspberrypi4':
    useNavx = False
    useVision = True
    videoTesting = True
    resizeVideo = False
    saveVideo = False
    findField = True
    navxTesting = 0
    visionTesting = 50
else:
    logFilename = '/home/pi/Team4121/Logs/Run_Log_' + timeString + '.txt'
    with open(logFilename, 'w') as log_file:
        log_file.write('run started on {}.\n'.format(datetime.datetime.now()))
        log_file.write('')
        log_file.close()

# Round all elements in a tuple, returning the formatted string
# tup is the tuple, p is the number of decimal places, and n is the number of digits in the number (to the left of the decimal)
def round_tuple(tup, p, n = 1):
    return '(' + ', '.join((("{:" + str(p + n + 2) + "." + str(p) + "f}").format(v) for v in tup)) + ')'

# Puts an iterable value to the network table
# Elements are added as name.0, name.1, name.2, ...
def put_iterable(table, name, tup):
    for (elem, count) in zip(tup, range(len(tup))):
        table.putNumber(name + "." + str(count), elem)

def unwrap_or(val, default):
    if val is None:
        return default
    else:
        return val

#Define main processing function
def main():

    global useNavx
    global timeString

    navxLoopCount = 0
    visionLoopCount = 0
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
    FRCWebCam.read_config_file(cameraFile)
    fieldCam = FRCWebCam('FIELD', timeString)
    tapeCam = FRCWebCam('TAPE', timeString)

    VisionBase.read_vision_file(visionFile)
    cubeLib = None
    coneLib = None
    tapeLib = None
    if findField:
        cubeLib = CubeVisionLibrary()
        coneLib = ConeVisionLibrary()

    if findTape:
        tapeLib = TapeRectVisionLibrary()

    #Create Navx object
    if useNavx:
        navx = FRCNavx('NavxStream')
        timeString = navx.get_raw_time()
        useNavx = not navx.poisoned
        
    #Create a blank array to hold the camera image
    fieldFrame = np.zeros((int(fieldCam.width), int(fieldCam.height), 3), np.uint8)

    #Open a log file
    logFilename = '/home/pi/Team4121/Logs/Run_Log_' + timeString + '.txt'
    with open(logFilename, 'w') as log_file:
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

        log_file.write("connected to table\n" if networkTablesConnected else "Failed to connect to table\n")

        #Start main processing loop
        while True:

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
                
                #Put gyro value in NetworkTables
                if networkTablesConnected:
                    navxTable.putNumber("GyroAngle", gyroAngle)
                    put_iterable(navxTable, "Orientation", navx.read_orientation())
                    put_iterable(navxTable, "Acceleration", navx.read_acceleration())
                    put_iterable(navxTable, "Velocity", navx.read_velocity())
                    put_iterable(navxTable, "Position", navx.read_position())

                
                navxLoopCount += 1
                if navxLoopCount + 1 == navxTesting:
                    print("angle: {:7.2f}\torientation: {}\tacceleration: {}\tvelocity: {}\tposition: {}".format(gyroAngle, round_tuple(navx.read_orientation(), 2, 3), round_tuple(navx.read_acceleration(), 4), round_tuple(navx.read_velocity(), 4), round_tuple(navx.read_position(), 4)))
                    navxLoopCount = 0
            else:
                gyroAngle = -9999  #Set default gyro angle
            
            

            ###################
            # Process Web Cam #
            ###################
            if findField:
                fieldFrame = fieldCam.read_frame()
                
                cubes = cubeLib.find_objects(fieldFrame, fieldCam.width, fieldCam.height, fieldCam.fov)
                    
                if len(cubes) >= 1:

                    cube = cubes[0]
                    cv.rectangle(fieldFrame, (cube.x, cube.y), ((cube.x + cube.w), (cube.y + cube.h)), (0, 0, 255), 2)
                    cv.putText(fieldFrame, "D: {:6.2f}".format(cube.distance), (cube.x + 10, cube.y + 15), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)
                    cv.putText(fieldFrame, "A: {:6.2f}".format(cube.angle), (cube.x + 10, cube.y + 30), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)
                    cv.putText(fieldFrame, "O: {:6.2f}".format(cube.offset), (cube.x + 10, cube.y + 45), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)

                    if len(cubes) >= 2:
                        cube = cubes[1]
                        cv.rectangle(fieldFrame, (cube.x, cube.y), ((cube.x + cube.w), (cube.y + cube.h)), (0, 255, 0), 2)
                        cv.putText(fieldFrame, "D: {:6.2f}".format(cube.distance), (cube.x + 10, cube.y + 15), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)
                        cv.putText(fieldFrame, "A: {:6.2f}".format(cube.angle), (cube.x + 10, cube.y + 30), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)
                        cv.putText(fieldFrame, "O: {:6.2f}".format(cube.offset), (cube.x + 10, cube.y + 45), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)

                        if len(cubes) >= 3:
                            cube = cubes[2]
                            cv.rectangle(fieldFrame, (cube.x, cube.y), ((cube.x + cube.w), (cube.y + cube.h)), (0, 255, 0), 2)
                            cv.putText(fieldFrame, "D: {:6.2f}".format(cube.distance), (cube.x + 10, cube.y + 15), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)
                            cv.putText(fieldFrame, "A: {:6.2f}".format(cube.angle), (cube.x + 10, cube.y + 30), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)
                            cv.putText(fieldFrame, "O: {:6.2f}".format(cube.offset), (cube.x + 10, cube.y + 45), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)

                cones = coneLib.find_objects(fieldFrame, fieldCam.width, fieldCam.height, fieldCam.fov)
                    
                if len(cones) >= 1:

                    cone = cones[0]
                    cv.rectangle(fieldFrame, (cone.x, cone.y), ((cone.x + cone.w), (cone.y + cone.h)), (0, 0, 255), 2)
                    cv.putText(fieldFrame, "D: {:6.2f}".format(cone.distance), (cone.x + 10, cone.y + 15), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)
                    cv.putText(fieldFrame, "A: {:6.2f}".format(cone.angle), (cone.x + 10, cone.y + 30), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)
                    cv.putText(fieldFrame, "O: {:6.2f}".format(cone.offset), (cone.x + 10, cone.y + 45), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)

                    if len(cones) >= 2:
                        cone = cones[1]
                        cv.rectangle(fieldFrame, (cone.x, cone.y), ((cone.x + cone.w), (cone.y + cone.h)), (0, 255, 0), 2)
                        cv.putText(fieldFrame, "D: {:6.2f}".format(cone.distance), (cone.x + 10, cone.y + 15), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)
                        cv.putText(fieldFrame, "A: {:6.2f}".format(cone.angle), (cone.x + 10, cone.y + 30), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)
                        cv.putText(fieldFrame, "O: {:6.2f}".format(cone.offset), (cone.x + 10, cone.y + 45), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)

                        if len(cones) >= 3:
                            cone = cones[2]
                            cv.rectangle(fieldFrame, (cone.x, cone.y), ((cone.x + cone.w), (cone.y + cone.h)), (0, 255, 0), 2)
                            cv.putText(fieldFrame, "D: {:6.2f}".format(cone.distance), (cone.x + 10, cone.y + 15), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)
                            cv.putText(fieldFrame, "A: {:6.2f}".format(cone.angle), (cone.x + 10, cone.y + 30), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)
                            cv.putText(fieldFrame, "O: {:6.2f}".format(cone.offset), (cone.x + 10, cone.y + 45), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 2)
                
                if videoTesting:
                    cv.imshow("Field", fieldFrame)
                
                if networkTablesConnected:
                    visionTable.putNumber("CubesFound", len(cubes))
                    if len(cubes) >= 1:
                        visionTable.putNumber("Cubes.0.distance", unwrap_or(cubes[0].distance, -9999.))
                        visionTable.putNumber("Cubes.0.angle", unwrap_or(cubes[0].angle, -9999.))
                        visionTable.putNumber("Cubes.0.offset", unwrap_or(cubes[0].offset, -9999.))
                        if len(cubes) >= 2:
                            visionTable.putNumber("Cubes.1.distance", unwrap_or(cubes[1].distance, -9999.))
                            visionTable.putNumber("Cubes.1.angle", unwrap_or(cubes[1].angle, -9999.))
                            visionTable.putNumber("Cubes.1.offset", unwrap_or(cubes[1].offset, -9999.))
                            if len(cubes) >= 3:
                                visionTable.putNumber("Cubes.2.distance", unwrap_or(cubes[2].distance, -9999.))
                                visionTable.putNumber("Cubes.2.angle", unwrap_or(cubes[2].angle, -9999.))
                                visionTable.putNumber("Cubes.2.offset", unwrap_or(cubes[2].offset, -9999.))
                    
                    visionTable.putNumber("ConesFound", len(cones))
                    if len(cones) >= 1:
                        visionTable.putNumber("Cones.0.distance", unwrap_or(cones[0].distance, -9999.))
                        visionTable.putNumber("Cones.0.angle", unwrap_or(cones[0].angle, -9999.))
                        visionTable.putNumber("Cones.0.offset", unwrap_or(cones[0].offset, -9999.))
                        if len(cones) >= 2:
                            visionTable.putNumber("Cones.1.distance", unwrap_or(cones[1].distance, -9999.))
                            visionTable.putNumber("Cones.1.angle", unwrap_or(cones[1].angle, -9999.))
                            visionTable.putNumber("Cones.1.offset", unwrap_or(cones[1].offset, -9999.))
                            if len(cones) >= 3:
                                visionTable.putNumber("Cones.2.distance", unwrap_or(cones[2].distance, -9999.))
                                visionTable.putNumber("Cones.2.angle", unwrap_or(cones[2].angle, -9999.))
                                visionTable.putNumber("Cones.2.offset", unwrap_or(cones[2].offset, -9999.))
            
            if findTape:
                tapeFrame = tapeCam.read_frame()

                tapes = tapeLib.find_objects(tapeFrame, tapeCam.width, tapeCam.height, tapeCam.fov)
                
                if len(tapes) >= 1:

                    tape = tapes[0]
                    cv.rectangle(tapeFrame, (tape.x, tape.y), ((tape.x + tape.w), (tape.y + tape.h)), (0, 0, 255), 2)
                    cv.putText(tapeFrame, "D: {:6.2f}".format(tape.distance), (tape.x + tape.w + 10, tape.y + 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    cv.putText(tapeFrame, "A: {:6.2f}".format(tape.angle), (tape.x + tape.w + 10, tape.y + 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    cv.putText(tapeFrame, "O: {:6.2f}".format(tape.offset), (tape.x + tape.w + 10, tape.y + 45), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    if len(tapes) >= 2:
                        tape = tapes[1]
                        cv.rectangle(tapeFrame, (tape.x, tape.y), ((tape.x + tape.w), (tape.y + tape.h)), (0, 255, 0), 2)
                        cv.putText(tapeFrame, "D: {:6.2f}".format(tape.distance), (tape.x + tape.w + 10, tape.y + 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        cv.putText(tapeFrame, "A: {:6.2f}".format(tape.angle), (tape.x + tape.w + 10, tape.y + 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        cv.putText(tapeFrame, "O: {:6.2f}".format(tape.offset), (tape.x + tape.w + 10, tape.y + 45), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                        if len(tapes) >= 3:
                            tape = tapes[2]
                            cv.rectangle(tapeFrame, (tape.x, tape.y), ((tape.x + tape.w), (tape.y + tape.h)), (0, 255, 0), 2)
                            cv.putText(tapeFrame, "D: {:6.2f}".format(tape.distance), (tape.x + tape.w + 10, tape.y + 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                            cv.putText(tapeFrame, "A: {:6.2f}".format(tape.angle), (tape.x + tape.w + 10, tape.y + 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                            cv.putText(tapeFrame, "O: {:6.2f}".format(tape.offset), (tape.x + tape.w + 10, tape.y + 45), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                            if len(tapes) >= 4:
                                tape = tapes[3]
                                cv.rectangle(tapeFrame, (tape.x, tape.y), ((tape.x + tape.w), (tape.y + tape.h)), (0, 255, 0), 2)
                                cv.putText(tapeFrame, "D: {:6.2f}".format(tape.distance), (tape.x + tape.w + 10, tape.y + 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                                cv.putText(tapeFrame, "A: {:6.2f}".format(tape.angle), (tape.x + tape.w + 10, tape.y + 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                                cv.putText(tapeFrame, "O: {:6.2f}".format(tape.offset), (tape.x + tape.w + 10, tape.y + 45), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                
                if videoTesting:
                    cv.imshow("Tapes", tapeFrame)
                
                if networkTablesConnected:
                    visionTable.putNumber("TapesFound", len(tapes))
                    if len(tapes) >= 1:
                        visionTable.putNumber("Tapes.0.distance", unwrap_or(tapes[0].distance, -9999.))
                        visionTable.putNumber("Tapes.0.angle", unwrap_or(tapes[0].angle, -9999.))
                        visionTable.putNumber("Tapes.0.offset", unwrap_or(tapes[0].offset, -9999.))
                        if len(tapes) >= 2:
                            visionTable.putNumber("Tapes.1.distance", unwrap_or(tapes[1].distance, -9999.))
                            visionTable.putNumber("Tapes.1.angle", unwrap_or(tapes[1].angle, -9999.))
                            visionTable.putNumber("Tapes.1.offset", unwrap_or(tapes[1].offset, -9999.))
                            if len(tapes) >= 3:
                                visionTable.putNumber("Tapes.2.distance", unwrap_or(tapes[2].distance, -9999.))
                                visionTable.putNumber("Tapes.2.angle", unwrap_or(tapes[2].angle, -9999.))
                                visionTable.putNumber("Tapes.2.offset", unwrap_or(tapes[2].offset, -9999.))
                                if len(tapes) >= 4:
                                    visionTable.putNumber("Tapes.3.distance", unwrap_or(tapes[3].distance, -9999.))
                                    visionTable.putNumber("Tapes.3.angle", unwrap_or(tapes[3].angle, -9999.))
                                    visionTable.putNumber("Tapes.3.offset", unwrap_or(tapes[3].offset, -9999.))
            
            visionLoopCount += 1
            if visionLoopCount + 1 == visionTesting:
                #print("found {} cubes".format(len(cubes)))
                visionLoopCount = 0
                

            #################################
            # Check for stopping conditions #
            #################################

            #Check for stop code from keyboard (for testing)
            if videoTesting:
                if cv.waitKey(1) == 27:
                    break

            #Check for stop code from network tables
            if networkTablesConnected: 
                robotStop = visionTable.getNumber("RobotStop", 0)
                if robotStop == 1 or not networkTablesConnected:
                    break

            #Pause before next analysis
            #time.sleep(0.066) #should give ~15 FPS

        #Close all open windows (for testing)
        if videoTesting == True:
            cv.destroyAllWindows()

        #Close the log file
        log_file.write('Run stopped on {}.'.format(datetime.datetime.now()))


#define main function
if __name__ == '__main__':
    main()
