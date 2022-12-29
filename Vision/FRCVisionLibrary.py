# -*- coding: utf-8 -*-
#!/usr/bin/env python3

####################################################################
#                                                                  #
#                       FRC Vision Library                         #
#                                                                  #
#  This class provides numerous methods and utilities for vision   #
#  processing during an FRC game.  The provided methods cover      #
#  finding standard game elements (balls, cubes, etc.) as well as  #
#  retroreflective vision targets in video frames.                 #
#                                                                  #
#  @Version: 1.0                                                   #
#  @Created: 2020-1-8                                              #
#  @Author: Team 4121                                              #
#                                                                  #
####################################################################

'''FRC Vision Library - Provides vision processing for game elements'''

# System imports
import os

# Module Imports
import cv2 as cv
import numpy as np 
import math

# Define the vision library class
class VisionLibrary:

    # Define class fields
    visionFile = ""
    ball1_values = {} 
    ball2_values = {}
    goal_values = {}
    tape_values = {}
    marker_values = {}


    # Define class initialization
    def __init__(self, visionfile):
        
        #Read in vision settings file
        VisionLibrary.visionFile = visionfile
        self.read_vision_file(VisionLibrary.visionFile)


    # Read vision settings file
    def read_vision_file(self, file):

        # Declare local variables
        value_section = ''
        new_section = False

        # Open the file and read contents
        try:
            
            # Open the file for reading
            in_file = open(file, 'r')
            
            # Read in all lines
            value_list = in_file.readlines()
            
            # Process list of lines
            for line in value_list:
                
                # Remove trailing newlines and whitespace
                clean_line = line.strip()

                # Split the line into parts
                split_line = clean_line.split(',')

                # Determine section of the file we are in
                if split_line[0].upper() == 'BALL1:':
                    value_section = 'BALL1'
                    new_section = True
                elif split_line[0].upper() == 'BALL2:':
                    value_section = 'BALL2'
                    new_section = True
                elif split_line[0].upper() == 'GOALTARGET:':
                    value_section = 'GOALTARGET'
                    new_section = True
                elif split_line[0].upper() == 'VISIONTAPE:':
                    value_section = 'VISIONTAPE'
                    new_section = True
                elif split_line[0].upper() == 'MARKER:':
                    value_section = 'MARKER'
                elif split_line[0] == '':
                    value_section = ''
                    new_section = True
                else:
                    new_section = False

                # Take action based on section
                if new_section == False:
                    if value_section == 'BALL1':
                        VisionLibrary.ball1_values[split_line[0].upper()] = split_line[1]
                    elif value_section == 'BALL2':
                        VisionLibrary.ball2_values[split_line[0].upper()] = split_line[1]
                    elif value_section == 'GOALTARGET':
                        VisionLibrary.goal_values[split_line[0].upper()] = split_line[1]
                    elif value_section == 'VISIONTAPE':
                        VisionLibrary.tape_values[split_line[0].upper()] = split_line[1]
                    elif value_section == 'MARKER':
                        VisionLibrary.marker_values[split_line[0].upper()] = split_line[1]
                    else:
                        new_section = True
        
        except FileNotFoundError:
            return False
        
        return True


    # Define basic image processing method for contours
    def process_image_contours(self, imgRaw, hsvMin, hsvMax, erodeDilate, useCanny):
        
        finalImg = ""

        # Blur image to remove noise
        blur = cv.GaussianBlur(imgRaw.copy(),(13,13),0)
        
        # Convert from BGR to HSV colorspace
        hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

        # Set pixels to white if in target HSV range, else set to black
        mask = cv.inRange(hsv, hsvMin, hsvMax)

        # Detect edges
        if useCanny == True:
            edges = cv.Canny(mask, 35, 125)
        else:
            edges = mask

        if erodeDilate:
            # Erode image to reduce background noise
            kernel = np.ones((3,3), np.uint8)
            erode = cv.erode(edges, kernel, iterations=2)
            
            # cv.imshow('erode', erode)
            
            # Dilate image to sharpen actual objects
            dilate = cv.dilate(erode, kernel, iterations=2)
            
            # cv.imshow('dilate', dilate)
            
            finalImg = dilate

        else:
            finalImg = edges
        
        # Find contours in mask
        contours, _ = cv.findContours(finalImg,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
    
        return contours

    # Define basic image processing method for edge detection
    def process_image_edges(self, imgRaw):

        # Blur image to remove noise
        blur = cv.GaussianBlur(imgRaw.copy(),(13,13),0)
        
        # Convert from BGR to HSV colorspace
        hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

        # Detect edges
        edges = cv.Canny(hsv, 35, 125)

        # Find contours using edges
        _, contours, _ = cv.findContours(edges.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)

        return contours


    # Find ball game pieces
    def detect_game_balls(self, imgRaw, cameraWidth, cameraHeight, cameraFOV, ballColor):

        # Define variables
        hMin = 0
        hmax = 0
        sMin = 0
        sMax = 0
        vMin = 0
        vMax = 0

        # # Read HSV values from dictionary and make tuples
        if ballColor == 1:
            hMin = int(VisionLibrary.ball1_values['HMIN'])
            hMax = int(VisionLibrary.ball1_values['HMAX'])
            sMin = int(VisionLibrary.ball1_values['SMIN'])
            sMax = int(VisionLibrary.ball1_values['SMAX'])
            vMin = int(VisionLibrary.ball1_values['VMIN'])
            vMax = int(VisionLibrary.ball1_values['VMAX'])
        else:
            hMin = int(VisionLibrary.ball2_values['HMIN'])
            hMax = int(VisionLibrary.ball2_values['HMAX'])
            sMin = int(VisionLibrary.ball2_values['SMIN'])
            sMax = int(VisionLibrary.ball2_values['SMAX'])
            vMin = int(VisionLibrary.ball2_values['VMIN'])
            vMax = int(VisionLibrary.ball2_values['VMAX']) 
        ballHSVMin = (hMin, sMin, vMin)
        ballHSVMax = (hMax, sMax, vMax)
        
        # Initialize variables
        distanceToBall = 0 #inches
        angleToBall = 0 #degrees
        ballOffset = 0
        screenPercent = 0
        ballsFound = 0
        ballData = []

        # Find contours in the mask and clean up the return style from OpenCV
        ballContours = self.process_image_contours(imgRaw, ballHSVMin, ballHSVMax,False)

        # Only proceed if at least one contour was found
        if len(ballContours) > 0:

            #Sort contours by area (reverse order so largest is first)
            sortedContours = sorted(ballContours, key=cv.contourArea, reverse=True)

            #Process each contour
            for contour in sortedContours:

                #Find enclosing circle
                ((x, y), radius) = cv.minEnclosingCircle(contour)

                #Get ball radius values
                minRadius = 0
                ballRadius = 0
                if ballColor == 1:
                    minRadius = VisionLibrary.ball1_values['MINRADIUS']
                    ballRadius = VisionLibrary.ball1_values['RADIUS']
                else:
                    minRadius = VisionLibrary.ball2_values['MINRADIUS']
                    ballRadius = VisionLibrary.ball2_values['RADIUS']

                #Proceed if circle meets minimum radius requirement
                if radius > float(minRadius):
            
                    #Calculate ball metrics
                    inches_per_pixel = float(ballRadius)/radius #set up a general conversion factor
                    distanceToTargetPlane = inches_per_pixel * (cameraWidth / (2 * math.tan(math.radians(cameraFOV))))
                    offsetInInches = inches_per_pixel * (x - cameraWidth / 2)
                    angleToBall = math.degrees(math.atan((offsetInInches / distanceToTargetPlane)))
                    distanceToBall = math.cos(math.radians(angleToBall)) * distanceToTargetPlane
                    screenPercent = math.pi * radius * radius / (cameraWidth * cameraHeight)
                    ballOffset = -offsetInInches

                    #Save values to dictionary
                    ballDataDict = {}
                    ballDataDict['x'] = x
                    ballDataDict['y'] = y
                    ballDataDict['radius'] = radius
                    ballDataDict['distance'] = distanceToBall
                    ballDataDict['angle'] = angleToBall
                    ballDataDict['offset'] = ballOffset
                    ballDataDict['percent'] = screenPercent

                    #Add dictionary to return list
                    ballData.append(ballDataDict)

                    #Increment ball count
                    ballsFound = ballsFound + 1
        
                else:

                    #No more contours meet criteria so break loop
                    break

        return ballsFound, ballData


    # Find game balls only (Ignore other objects of same color)
    def detect_game_balls_only(self, imgRaw, cameraWidth, cameraHeight, cameraFOV, ballColor):

        # Define variables
        hMin = 0
        hmax = 0
        sMin = 0
        sMax = 0
        vMin = 0
        vMax = 0

        # # Read HSV values from dictionary and make tuples
        if ballColor == 1:
            hMin = int(VisionLibrary.ball1_values['HMIN'])
            hMax = int(VisionLibrary.ball1_values['HMAX'])
            sMin = int(VisionLibrary.ball1_values['SMIN'])
            sMax = int(VisionLibrary.ball1_values['SMAX'])
            vMin = int(VisionLibrary.ball1_values['VMIN'])
            vMax = int(VisionLibrary.ball1_values['VMAX'])
        else:
            hMin = int(VisionLibrary.ball2_values['HMIN'])
            hMax = int(VisionLibrary.ball2_values['HMAX'])
            sMin = int(VisionLibrary.ball2_values['SMIN'])
            sMax = int(VisionLibrary.ball2_values['SMAX'])
            vMin = int(VisionLibrary.ball2_values['VMIN'])
            vMax = int(VisionLibrary.ball2_values['VMAX']) 
        ballHSVMin = (hMin, sMin, vMin)
        ballHSVMax = (hMax, sMax, vMax)
        
        # Initialize variables
        distanceToBall = 0 #inches
        angleToBall = 0 #degrees
        ballOffset = 0
        screenPercent = 0
        ballsFound = 0
        ballData = []
        ballTolerance = 0.1

        # Find contours in the mask and clean up the return style from OpenCV
        ballContours = self.process_image_contours(imgRaw, ballHSVMin, ballHSVMax, False, True)

        # Only proceed if at least one contour was found
        if len(ballContours) > 0:

            #Sort contours by area (reverse order so largest is first)
            sortedContours = sorted(ballContours, key=cv.contourArea, reverse=True)

            #Process each contour
            for contour in sortedContours:

                # Find bounding rectangle
                contourX, contourY, contourW, contourH = cv.boundingRect(contour)

                # Determine if this is a ball (check aspect ratio)
                aspectRatio = contourH / contourW
                if (aspectRatio >= 1 - ballTolerance) and (aspectRatio <= 1 + ballTolerance):

                    #Get ball radius values
                    minRadius = 0
                    ballRadius = 0
                    if ballColor == 1:
                        minRadius = VisionLibrary.ball1_values['MINRADIUS'] #in pixel units
                        ballRadius = VisionLibrary.ball1_values['RADIUS'] #in inches
                    else:
                        minRadius = VisionLibrary.ball2_values['MINRADIUS'] #in pixel units
                        ballRadius = VisionLibrary.ball2_values['RADIUS']

                    #Proceed if circle meets minimum radius requirement
                    radius = contourW / 2
                    x = contourX + contourW / 2 #x of center of circle [in pixels?]
                    y = contourY + contourH / 2 #y of center of circle
                    if radius > float(minRadius): #in pixel units
                
                        #Calculate ball metrics
                        inches_per_pixel = float(ballRadius)/radius #set up a general conversion factor [ballRadius{in inches} / radius{in pixels}]
                        distanceToTargetPlane = inches_per_pixel * (cameraWidth / (2 * math.tan(math.radians(cameraFOV)))) #distanceToBall = targetPlaneSize/(2*tangent of half of viewing angle
                        offsetInInches = inches_per_pixel * (x - cameraWidth / 2)
                        angleToBall = math.degrees(math.atan((offsetInInches / distanceToTargetPlane)))
                        distanceToBall = math.cos(math.radians(angleToBall)) * distanceToTargetPlane
                        screenPercent = math.pi * radius * radius / (cameraWidth * cameraHeight)
                        ballOffset = -offsetInInches

                        #Save values to dictionary
                        ballDataDict = {}
                        ballDataDict['x'] = x
                        ballDataDict['y'] = y
                        ballDataDict['radius'] = radius
                        ballDataDict['distance'] = distanceToBall
                        ballDataDict['angle'] = angleToBall
                        ballDataDict['offset'] = ballOffset
                        ballDataDict['percent'] = screenPercent

                        #Add dictionary to return list
                        ballData.append(ballDataDict)

                        #Increment ball count
                        ballsFound = ballsFound + 1
            
                    else:

                        #No more contours meet criteria so break loop
                        break

        return ballsFound, ballData

    
    #find game field markers
    def detect_field_marker(self, imgRaw, cameraWidth, cameraHeight, cameraFOV):
        
        # Read HSV values from dictionary and make tuples
        hMin = int(VisionLibrary.marker_values['HMIN'])
        hMax = int(VisionLibrary.marker_values['HMAX'])
        sMin = int(VisionLibrary.marker_values['SMIN'])
        sMax = int(VisionLibrary.marker_values['SMAX'])
        vMin = int(VisionLibrary.marker_values['VMIN'])
        vMax = int(VisionLibrary.marker_values['VMAX'])
        markerHSVMin = (hMin, sMin, vMin)
        markerHSVMax = (hMax, sMax, vMax)

         # Initialize values to be returned
        markerArea = 0 #px
        markerX = -1 #px
        markerY = -1 #px
        markerW = 0
        markerH = 0
        distanceToMarker = 0 #inches
        angleToMarker = 0 #degrees
        screenPercent = 0
        markersFound = 0
        markerData = []

        # Calculate Ratio TOL
        ratioMax = float(VisionLibrary.marker_values['TARGETRATIO']) + float(VisionLibrary.marker_values['RATIOTOL'])
        ratioMin = float(VisionLibrary.marker_values['TARGETRATIO']) - float(VisionLibrary.marker_values['RATIOTOL'])
        
        #finding marker contours
        markerContours = self.process_image_contours(imgRaw, markerHSVMin, markerHSVMax, False, True)

        # Only proceed if at least one contour was found
        if len(markerContours) > 0:

            #Sort contours by area (reverse order so largest is first)
            sortedContours = sorted(markerContours, key=cv.contourArea, reverse=True)

            #Process each contour
            for contour in sortedContours:

                # Find bounding rectangle
                markerX, markerY, markerW, markerH = cv.boundingRect(contour)

                # Check area
                area = (markerW * markerH)
                if area > int(VisionLibrary.marker_values['MINAREA']):

                    # Check for ratio
                    #if ((markerH / markerW) > ratioMin) and ((markerH / markerW) < ratioMax):

                    # Marker distance calculations
                    inches_per_pixel = float(VisionLibrary.marker_values['HEIGHT'])/markerH #set up a general conversion factor
                    distanceToTargetPlane = inches_per_pixel * (cameraWidth / (2 * math.tan(math.radians(cameraFOV))))
                    offsetInInches = inches_per_pixel * (markerX - cameraWidth / 2)
                    angleToMarker = math.degrees(math.atan((offsetInInches / distanceToTargetPlane)))
                    distanceToMarker = math.cos(math.radians(angleToMarker)) * distanceToTargetPlane
                    screenPercent = area / (cameraWidth * cameraHeight)
                    markerOffset = -offsetInInches

                    #Save values to dictionary
                    markerDataDict = {}
                    markerDataDict['x'] = markerX
                    markerDataDict['y'] = markerY
                    markerDataDict['w'] = markerW
                    markerDataDict['h'] = markerH
                    markerDataDict['distance'] = distanceToMarker
                    markerDataDict['angle'] = angleToMarker
                    markerDataDict['offset'] = markerOffset
                    markerDataDict['percent'] = screenPercent

                    #Add dictionary to return list
                    markerData.append(markerDataDict)

                    #Increment ball count
                    markersFound = markersFound + 1
                        
                else:

                    #No more contours meet criteria so break loop
                    break

        return markersFound, markerData
       


    # Define general tape detection method (rectangle good for generic vision tape targets)
    def detect_tape_rectangle(self, imgRaw, imageWidth, imageHeight, cameraFOV, cameraFocalLength, cameraMountAngle, cameraMountHeight):

        # Read HSV values from dictionary and make tupples
        hMin = int(VisionLibrary.tape_values['HMIN'])
        hMax = int(VisionLibrary.tape_values['HMAX'])
        sMin = int(VisionLibrary.tape_values['SMIN'])
        sMax = int(VisionLibrary.tape_values['SMAX'])
        vMin = int(VisionLibrary.tape_values['VMIN'])
        vMax = int(VisionLibrary.tape_values['VMAX'])
        tapeHSVMin = (hMin, sMin, vMin)
        tapeHSVMax = (hMax, sMax, vMax)

        # Initialize processing values
        targetX = 1000
        targetY = 1000
        targetW = 1000
        targetH = 1000
        aspectRatio = 0
        centerOffset = 0
        cameraAngle = 0
        actualVertAngle = 0
        botAngle = 0
        apparentTapeWidth = 0
        distanceArg = 0
        straightLineDistance = 1000
        distanceToTape = 1000
        distanceToWall = 1000
        horizAngleToTape = 0
        vertAngleToTape = 0
        inchesPerPixel = 0
        horizOffsetPixels = 0
        horizOffsetInInches = 0
        vertOffsetPixels = 0
        vertOffsetInInches = 0
        rect = None
        box = None

        # Initialize flags
        foundTape = False
        targetLock = False

        goalHeight = 90.0

        # Return dictionary
        tapeCameraValues = {}
        tapeRealWorldValues = {}
        
        # Find alignment tape in image
        tapeContours = self.process_image_contours(imgRaw, tapeHSVMin, tapeHSVMax, True, False)
  
        # Continue with processing if alignment tape found
        if len(tapeContours) > 0:

            # Find the largest contour and check it against the mininum tape area
            largestContour = max(tapeContours, key=cv.contourArea)
                        
            if cv.contourArea(largestContour) > int(VisionLibrary.tape_values['MINAREA']):
                
                # Find horizontal rectangle
                targetX, targetY, targetW, targetH = cv.boundingRect(largestContour)

                # Calculate aspect ratio
                aspectRatio = targetW / targetH

                # Find angled rectangle
                rect = cv.minAreaRect(largestContour)#((x, y), (h, w), angle)
                box = cv.boxPoints(rect)
                box = np.int0(box)

                # Find angle of bot to target
                angle = rect[2]
                if abs(angle) < 45:
                    cameraAngle = abs(angle)
                else:
                    cameraAngle = 90 - abs(angle)
                botAngle = 2 * cameraAngle

                # Set flag
                foundTape = True
                
            # Calculate real world values of found tape
            if foundTape:
                
                # Adjust tape size for robot angle
                apparentTapeWidth = float(VisionLibrary.tape_values['TAPEWIDTH']) * math.cos(math.radians(botAngle))
                
                # Calculate inches per pixel conversion factor
                inchesPerPixel = apparentTapeWidth / targetW

                # Find tape offsets
                horizOffsetPixels = (targetX + targetW/2) - imageWidth / 2
                horizOffsetInInches = inchesPerPixel * horizOffsetPixels
                vertOffsetPixels = (imageHeight / 2) - (targetY - targetH/2)
                vertOffsetInInches = inchesPerPixel * vertOffsetPixels
                centerOffset = -horizOffsetInInches
                
                # Calculate distance to tape
                straightLineDistance = apparentTapeWidth * cameraFocalLength / targetW
                distanceArg = math.pow(straightLineDistance, 2) - math.pow((float(VisionLibrary.tape_values['GOALHEIGHT']) - cameraMountHeight),2)
                if (distanceArg > 0):
                    distanceToTape = math.sqrt(distanceArg)
                distanceToWall = distanceToTape / math.cos(math.radians(botAngle))                

                # Find tape offsets
                horizAngleToTape = math.degrees(math.atan((horizOffsetInInches / distanceToTape)))
                vertAngleToTape = math.degrees(math.atan((vertOffsetInInches / distanceToTape)))

                # Determine if we have target lock
                if abs(horizOffsetInInches) <= float(VisionLibrary.tape_values['LOCKTOLERANCE']):
                    targetLock = True

        # Fill return dictionary
        tapeCameraValues['TargetX'] = targetX
        tapeCameraValues['TargetY'] = targetY
        tapeCameraValues['TargetW'] = targetW
        tapeCameraValues['TargetH'] = targetH
        tapeCameraValues['IPP'] = inchesPerPixel
        tapeCameraValues['Offset'] = horizOffsetPixels
        tapeRealWorldValues['AspectRatio'] = aspectRatio
        tapeRealWorldValues['CenterOffset'] = centerOffset
        tapeRealWorldValues['StraightDistance'] = straightLineDistance
        tapeRealWorldValues['TapeDistance'] = distanceToTape
        tapeRealWorldValues['WallDistance'] = distanceToWall
        tapeRealWorldValues['HAngle'] = horizAngleToTape
        tapeRealWorldValues['VAngle'] = vertAngleToTape
        tapeRealWorldValues['TargetRotation'] = cameraAngle
        tapeRealWorldValues['BotAngle'] = botAngle
        tapeRealWorldValues['ApparentWidth'] = apparentTapeWidth
        tapeRealWorldValues['VertOffset'] = vertOffsetInInches

        return tapeCameraValues, tapeRealWorldValues, foundTape, targetLock, rect, box

    
    # Define general tape detection method (rectangle good for generic vision tape targets)
    def detect_black_tape_rectangle(self, imgRaw, imageWidth, imageHeight, cameraFOV, cameraFocalLength, cameraMountAngle, cameraMountHeight):

        # Read HSV values from dictionary and make tupples
        hMin = int(VisionLibrary.tape_values['HMIN'])
        hMax = int(VisionLibrary.tape_values['HMAX'])
        sMin = int(VisionLibrary.tape_values['SMIN'])
        sMax = int(VisionLibrary.tape_values['SMAX'])
        vMin = int(VisionLibrary.tape_values['VMIN'])
        vMax = int(VisionLibrary.tape_values['VMAX'])
        tapeHSVMin = (hMin, sMin, vMin)
        tapeHSVMax = (hMax, sMax, vMax)

        # Initialize processing values
        targetX = 1000 
        targetY = 1000
        targetW = 1000
        targetH = 1000
        aspectRatio = 0
        centerOffset = 0
        cameraAngle = 0
        actualVertAngle = 0
        botAngle = 0
        apparentTapeWidth = 0
        distanceArg = 0
        straightLineDistance = 1000
        distanceToTape = 1000
        distanceToWall = 1000
        horizAngleToTape = 0
        vertAngleToTape = 0
        inchesPerPixel = 0
        horizOffsetPixels = 0
        horizOffsetInInches = 0
        vertOffsetPixels = 0
        vertOffsetInInches = 0
        rect = None
        box = None

        # Initialize flags
        foundTape = False
        targetLock = False

        goalHeight = 98.0

        # Return dictionary
        tapeCameraValues = {}
        tapeRealWorldValues = {}
        
        # Find alignment tape in image
        tapeContours = self.process_image_contours(imgRaw, tapeHSVMin, tapeHSVMax, False, True)
  
        # Continue with processing if alignment tape found
        if len(tapeContours) > 0:

            # Find the largest contour and check it against the mininum tape area
            largestContour = max(tapeContours, key=cv.contourArea)
                        
            if cv.contourArea(largestContour) > int(VisionLibrary.tape_values['MINAREA']):
                
                # Find horizontal rectangle
                targetX, targetY, targetW, targetH = cv.boundingRect(largestContour)

                # Calculate aspect ratio
                aspectRatio = targetW / targetH

                # Find angled rectangle
                rect = cv.minAreaRect(largestContour)#((x, y), (h, w), angle)
                box = cv.boxPoints(rect)
                box = np.int0(box)

                # Find angle of bot to target
                angle = rect[2]
                if abs(angle) < 45:
                    cameraAngle = abs(angle)
                else:
                    cameraAngle = 90 - abs(angle)
                botAngle = 2 * cameraAngle

                # Set flag
                foundTape = True
                
            # Calculate real world values of found tape
            if foundTape:
                
                # Adjust tape size for robot angle
                apparentTapeWidth = float(VisionLibrary.tape_values['TAPEWIDTH']) * math.cos(math.radians(botAngle))
                
                # Calculate inches per pixel conversion factor
                inchesPerPixel = apparentTapeWidth / targetW

                # Find tape offsets
                horizOffsetPixels = (targetX + targetW/2) - imageWidth / 2
                horizOffsetInInches = inchesPerPixel * horizOffsetPixels
                vertOffsetPixels = (imageHeight / 2) - (targetY - targetH/2)
                vertOffsetInInches = inchesPerPixel * vertOffsetPixels
                centerOffset = -horizOffsetInInches
                
                # Calculate distance to tape
                straightLineDistance = apparentTapeWidth * cameraFocalLength / targetW
                distanceArg = math.pow(straightLineDistance, 2) - math.pow((float(VisionLibrary.tape_values['GOALHEIGHT']) - cameraMountHeight),2)
                if (distanceArg > 0):
                    distanceToTape = math.sqrt(distanceArg)
                distanceToWall = distanceToTape / math.cos(math.radians(botAngle))                

                # Find tape offsets
                horizAngleToTape = math.degrees(math.atan((horizOffsetInInches / distanceToTape)))
                vertAngleToTape = math.degrees(math.atan((vertOffsetInInches / distanceToTape)))

                # Determine if we have target lock
                if abs(horizOffsetInInches) <= float(VisionLibrary.tape_values['LOCKTOLERANCE']):
                    targetLock = True

        # Fill return dictionary
        tapeCameraValues['TargetX'] = targetX
        tapeCameraValues['TargetY'] = targetY
        tapeCameraValues['TargetW'] = targetW
        tapeCameraValues['TargetH'] = targetH
        tapeCameraValues['IPP'] = inchesPerPixel
        tapeCameraValues['Offset'] = horizOffsetPixels
        tapeRealWorldValues['AspectRatio'] = aspectRatio
        tapeRealWorldValues['CenterOffset'] = centerOffset
        tapeRealWorldValues['StraightDistance'] = straightLineDistance
        tapeRealWorldValues['TapeDistance'] = distanceToTape
        tapeRealWorldValues['WallDistance'] = distanceToWall
        tapeRealWorldValues['HAngle'] = horizAngleToTape
        tapeRealWorldValues['VAngle'] = vertAngleToTape
        tapeRealWorldValues['TargetRotation'] = cameraAngle
        tapeRealWorldValues['BotAngle'] = botAngle
        tapeRealWorldValues['ApparentWidth'] = apparentTapeWidth
        tapeRealWorldValues['VertOffset'] = vertOffsetInInches

        return tapeCameraValues, tapeRealWorldValues, foundTape, targetLock, rect, box

 # Define general tape detection method (rectangle good for generic vision tape targets)
    def detect_four_vision_tapes_rectangle(self, imgRaw, imageWidth, imageHeight, cameraFOV, cameraFocalLength, cameraMountAngle, cameraMountHeight):

        # Read HSV values from dictionary and make tupples
        hMin = int(VisionLibrary.tape_values['HMIN'])
        hMax = int(VisionLibrary.tape_values['HMAX'])
        sMin = int(VisionLibrary.tape_values['SMIN'])
        sMax = int(VisionLibrary.tape_values['SMAX'])
        vMin = int(VisionLibrary.tape_values['VMIN'])
        vMax = int(VisionLibrary.tape_values['VMAX'])
        tapeHSVMin = (hMin, sMin, vMin)
        tapeHSVMax = (hMax, sMax, vMax)

        # Initialize processing values
        targetX = 1000 
        targetY = 1000
        targetW = 1000
        targetH = 1000
        aspectRatio = 0
        centerOffset = 0
        cameraAngle = 0
        actualVertAngle = 0
        botAngle = 0
        apparentTapeWidth = 0
        distanceArg = 0
        straightLineDistance = 1000
        distanceToTape = 1000
        distanceToWall = 1000
        horizAngleToTape = 0
        vertAngleToTape = 0
        inchesPerPixel = 0
        horizOffsetPixels = 0
        horizOffsetInInches = 0
        vertOffsetPixels = 0
        vertOffsetInInches = 0
        rect = None
        box = None


        # Initialize flags
        foundTape = False
        targetLock = False

        # Return dictionary
        tapeCameraValues = {}
        tapeRealWorldValues = {}
        
        # Find alignment tape in image
        tapeContours = self.process_image_contours(imgRaw, tapeHSVMin, tapeHSVMax, False, False)

        # Continue with processing if alignment tape found
        if len(tapeContours) >= 3: #AT LEAST three
            
            #Process each contour
            firstContour = True
            minOffset = 0
            for contour in tapeContours:
              
                # Find horizontal rectangle
                rectX, rectY, rectW, rectH = cv.boundingRect(contour) 

                if (rectW * rectH) > int(VisionLibrary.tape_values['MINAREA']):
                    
                    # Find offset of rectangle from center of image
                    rectOffset = abs((rectX + rectW/2) - imageWidth / 2) #from parameter

                    if firstContour == True:

                        minOffset = rectOffset
                        firstContour = False
                        targetX = rectX
                        targetY = rectY
                        targetW = rectW
                        targetH = rectH

                    else:

                        if rectOffset < minOffset:

                            minOffset = rectOffset
                            targetX = rectX
                            targetY = rectY
                            targetW = rectW
                            targetH = rectH

                    #  Calculate aspect ratio
                    #  aspectRatio = generalRectW / generalRectH

                    # # Find angled rectangle
                    # rect = cv.minAreaRect(contour)#returns ((x, y), (h, w), angle)
                    # #box = cv.boxPoints(rect)
                    # #box = np.int0(box)

                    # # Find angle of bot to target
                    # angle = rect[2]
                    # if abs(angle) < 45:
                    #     cameraAngle = abs(angle)
                    # else:
                    #      cameraAngle = 90 - abs(angle)
                    # botAngle = 2 * cameraAngle

                    # Set flag
                    foundTape = True 
            
            # Calculate real world values of found tape
            if foundTape:
                                
                # Calculate inches per pixel conversion factor
                inchesPerPixel = float(VisionLibrary.tape_values['TAPEWIDTH']) / targetW

                # Find tape offsets
                horizOffsetPixels = (targetX + targetW/2) - imageWidth / 2 #from parameter
                horizOffsetInInches = inchesPerPixel * horizOffsetPixels
                vertOffsetPixels = (imageHeight / 2) - (targetY - targetH/2)
                vertOffsetInInches = inchesPerPixel * vertOffsetPixels
                centerOffset = -horizOffsetInInches
                
                # Calculate distance to tape
                straightLineDistance = float(VisionLibrary.tape_values['TAPEWIDTH']) * cameraFocalLength / targetW
                distanceArg = math.pow(straightLineDistance, 2) - math.pow((float(VisionLibrary.tape_values['GOALHEIGHT']) - cameraMountHeight),2)
                if (distanceArg > 0):
                    distanceToTape = math.sqrt(distanceArg)
                else:
                    distanceToTape = straightLineDistance
                distanceToWall = distanceToTape                

                # Find tape offsets
                horizAngleToTape = math.degrees(math.atan((horizOffsetInInches / distanceToTape)))
                vertAngleToTape = math.degrees(math.atan((vertOffsetInInches / distanceToTape)))

                # Determine if we have target lock
                if abs(horizOffsetInInches) <= float(VisionLibrary.tape_values['LOCKTOLERANCE']):
                    targetLock = True

        # Fill return dictionary
        tapeCameraValues['TargetX'] = targetX
        tapeCameraValues['TargetY'] = targetY
        tapeCameraValues['TargetW'] = targetW
        tapeCameraValues['TargetH'] = targetH
        tapeCameraValues['IPP'] = inchesPerPixel
        tapeCameraValues['Offset'] = horizOffsetPixels
        tapeRealWorldValues['AspectRatio'] = aspectRatio
        tapeRealWorldValues['CenterOffset'] = centerOffset
        tapeRealWorldValues['StraightDistance'] = straightLineDistance
        tapeRealWorldValues['TapeDistance'] = distanceToTape
        tapeRealWorldValues['WallDistance'] = distanceToWall
        tapeRealWorldValues['HAngle'] = horizAngleToTape
        tapeRealWorldValues['VAngle'] = vertAngleToTape
        tapeRealWorldValues['TargetRotation'] = cameraAngle
        tapeRealWorldValues['BotAngle'] = 0
        tapeRealWorldValues['ApparentWidth'] = apparentTapeWidth
        tapeRealWorldValues['VertOffset'] = vertOffsetInInches

        return tapeCameraValues, tapeRealWorldValues, foundTape, targetLock
