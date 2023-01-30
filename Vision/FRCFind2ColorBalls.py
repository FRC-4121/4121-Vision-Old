#####################################################################
#                                                                   #
#                        FRC Vision Library                         #
#                  Find 2 Different Colored Balls                   #
#                                                                   #
#  This class provides a method for detecting game balls that come  #
#  in two different colors.  This class inherits from the vision    #
#  base class (FRCVisionBase).                                      #
#                                                                   #
#  Inherits from:  FRCVisionBase                                    #
#                                                                   #
#  @Version: 1.0                                                    #
#  @Created: 2023-1-30                                              #
#  @Author: Team 4121                                               #
#                                                                   #
#####################################################################

'''FRC 2 Ball Detector Library - Provides vision processing for finding game balls of two colors'''

# System imports
import os

# Module Imports
import cv2 as cv
import numpy as np 
import math

# FRC Library Imports
from FRCVisionBase import VisionBase

# Define the class
class FindBalls(VisionBase):

    # Initialize the class
    def __init__(self, visionfile):

        super().__init__(visionfile)