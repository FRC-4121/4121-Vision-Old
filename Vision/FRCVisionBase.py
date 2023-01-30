##################################################################
#                                                                #
#                   FRC Vision Library Base                      #
#                                                                #
#  This class is a base class for all FRC game specific vision   #
#  libraries.  The base class provides methods which are common  #
#  to all game object identification tasks.                      #
#                                                                #
#  @Version: 1.0                                                 #
#  @Created: 2023-1-29                                            #
#  @Author: Team 4121                                            #
#                                                                #
##################################################################

'''FRC Vision Base Class - Provides common vision processing for game elements'''

# Module Imports
import cv2 as cv
import numpy as np 
import math

# Define the class
class VisionBase:

    # Define class fields
    visionFile = ""
    fileSection = ""
    objectValues = {}


    # Class Initialization method
    # Reads the contents of the supplied vision settings file
    def __init__(self, visionfile):

        #Read in vision settings file
        self.visionFile = visionfile
        self.read_vision_file(self.visionFile)


    # Read vision settings file
    def read_vision_file(self, file, section):

        # Declare local variables
        value_section = ''
        new_section = False

        # Open the file and read contents
        try:
            
            # Open the file for reading
            in_file = open(file, 'r')
            
            # Read in all lines
            value_list = in_file.readlines()
            
            # Process all lines
            for line in value_list:
                
                # Remove trailing newlines and whitespace
                clean_line = line.strip()

                # Split the line into parts
                split_line = clean_line.split(',')

                # Determine which section of the file we are in
                if split_line[0].upper() == section:
                    sectionFound = True
                else:
                    sectionFound - False

                # Read in the object's values from the target section
                if sectionFound:
                    self.objectValues[split_line[0].upper()] = split_line[1]
        
        except FileNotFoundError:

            return False
        
        return True
    

    # Define basic image processing method for finding contours
    # Converts image from BGR color space to HSV and then applies a mask
    # based on "learned" HSV values from the config file.
    # Edge detection can also be imployed before contours are found and returned.
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
    

    # Method for finding game objects
    # Generic method which will be overidden by child classes
    def find_object(self, imgRaw, cameraWidth, cameraHeight, cameraFOV, objectColor):

        pass 

