# -*- coding: utf-8 -*-
#!/usr/bin/env python3

######################################################################
#                                                                    #
#                        FRC Camera Library                          #
#                                                                    #
#  This class provides methods and utilities for web cameras used    #
#  for vision processing during an FRC game.  Reading of the webcam  #
#  frames is threaded for improved performance.                      #
#                                                                    #
# @Version: 2.0                                                      #
# @Created: 2020-02-07                                               #
# @Revised: 2021-02-16                                               #
# @Author: Team 4121                                                 #
#                                                                    #
######################################################################

"""FRC Camera Library - Provides threaded camera methods and utilities"""

# System imports
import sys
import os
import logging

# Module Imports
import cv2 as cv
import numpy as np
from threading import Thread
from cscore import CvSource, VideoMode

#Set up basic logging
logging.basicConfig(level=logging.DEBUG)

# Set global variables
calibration_dir = "/home/pi/Team4121/Config"

def find_cams(port: int):
    file = f"/sys/devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/0000:01:00.0/usb1/1-1/1-1.{port}/1-1.{port}:1.0/video4linux"
    if os.path.exists(file):
        files = [int(x[5:]) for x in os.listdir(file) if x[:5] == "video" and x[5:].isnumeric()]
        files.sort()
        if len(files) > 0:
            return files[0]


# Define the web camera class
class FRCWebCam:
    config = {"": {}}
    init = False
    # Define initialization
    def __init__(self, name, timestamp, videofile = None, csname = None):
        self.name = name
        port = self.get_config("PORT", None)
        if port is not None:
            port = find_cams(port)
        if port is None:
            self.device_id = self.get_config("ID", "0")
            self.device_id = int(self.device_id) if self.device_id.isnumeric() else self.device_id
        else:
            self.device_id = port    

        #Open a log file
        logFilename = "/home/pi/Team4121/Logs/Webcam_Log_{}_{}.txt".format(self.name, timestamp)
        if videofile is None:
            videofile = "{}_{}".format(name, timestamp)
        self.log_file = open(logFilename, "w")
        self.log_file.write("Initializing webcam: {}\n".format(self.name))
        if self.device_id == "":
            print("Device ID not specified for camera {}".format(self.name))
            self.log_file.write("Device ID not specified for camera {}\n".format(self.name))
            return
        # Initialize instance variables
        self.undistort_img = False

        # Store frame size
        self.height = int(self.get_config("HEIGHT", 240))
        self.width = int(self.get_config("WIDTH", 320))
        self.fov = float(self.get_config("FOV", 0.0))
        self.fps = int(self.get_config("FPS", 15))
        self.streamRes = int(self.get_config("STREAM_RES", 1))

        # Set up web camera
        #self.camStream = cv.VideoCapture(self.device_id)
        self.camStream = cv.VideoCapture(self.device_id)
        self.camStream.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        self.camStream.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        self.camStream.set(cv.CAP_PROP_BRIGHTNESS, float(self.get_config("BRIGHTNESS", 0)))
        self.camStream.set(cv.CAP_PROP_EXPOSURE, int(self.get_config("EXPOSURE", 0)))
        self.camStream.set(cv.CAP_PROP_FPS, self.fps)

        # Set up video writer
        self.videoFilename = "/home/pi/Team4121/Videos/" + videofile + ".avi"
        fourcc = cv.VideoWriter_fourcc(*"MJPG")
        self.camWriter = cv.VideoWriter(self.videoFilename, fourcc, self.fps, (self.width, self.height))

        try:
            self.camWriter.open(self.videoFilename, self.fourcc, 
                                float(self.get_config("FPS", 15)), 
                                (self.width, self.height),
                                True)
        except:
            self.log_file.write("Error opening video writer for file: {}\n".format(self.videoFilename))
        
        if self.camWriter.isOpened():
            self.log_file.write("Video writer is open\n")
        else:
            self.log_file.write("Video writer is NOT open\n")

        # Make sure video capture is opened
        if self.camStream.isOpened() == False:
            print("Camera stream is not open")
            self.camStream.open(self.device_id)

        # Initialize blank frames
        self.frame = np.zeros(shape=(self.width, self.height, 3), dtype=np.uint8)

        # Grab an initial frame
        self.grabbed, self.frame = self.camStream.read()

        # Name the stream
        self.name = name

        # Initialize stop flag
        self.stopped = False

        # Read camera calibration files
        cam_matrix_file = calibration_dir + "/Camera_Matrix_Cam" + str(self.device_id) + ".txt"
        cam_coeffs_file = calibration_dir + "/Distortion_Coeffs_Cam" + str(self.device_id) + ".txt"
        if os.path.isfile(cam_matrix_file) == True and os.path.isfile(cam_coeffs_file) == True:
            self.cam_matrix = np.loadtxt(cam_matrix_file)
            self.distort_coeffs = np.loadtxt(cam_coeffs_file)
            self.undistort_img = True
        
        if csname is not None:
            self.cvs = CvSource(csname, VideoMode.PixelFormat.kBGR, self.width // self.streamRes, self.height // self.streamRes, self.fps)
        else:
            self.cvs = None

        # Log init complete message
        self.log_file.write("Webcam initialization complete\n")

    @staticmethod
    def read_config_file(file, reload = False):
        if FRCWebCam.init and not reload:
            return True
        FRCWebCam.init = True
        # Declare local variables
        value_section = ''
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
                if len(clean_line) == 0:
                    continue

                if clean_line[0] == '#':
                    continue
                # Split the line into parts
                split_line = clean_line.split('=')
                # Determine section of the file we are in
                upper_line = split_line[0].upper()
                
                if upper_line[-1] == ':':
                    value_section = upper_line[:-1]
                    if not value_section in FRCWebCam.config:
                        FRCWebCam.config[value_section] = {}
                elif split_line[0] == '':
                    value_section = ''
                    if not value_section in FRCWebCam.config:
                        FRCWebCam.config[value_section] = {}
                else:
                    FRCWebCam.config[value_section][split_line[0].upper()] = split_line[1]
        
        except FileNotFoundError:
            return False
        
        return True

    def get_config(self, name, default):
        if self.name in FRCWebCam.config:
            cfg = FRCWebCam.config[self.name]
            if name in cfg:
                return cfg[name]
        cfg = FRCWebCam.config[""]
        if name in cfg:
            return cfg[name]
        return default


    # Define camera thread start method
    def start_camera_thread(self):

        # Define camera thread
        camThread = Thread(target=self.update, name=self.name, args=())
        camThread.daemon = True
        camThread.start()

        return self


    # Define camera thread stop method
    def stop_camera_thread(self):

        # Set stop flag
        self.stopped = True    


    # Define threaded update method
    def update(self):

        # Main thread loop
        while True:

            # Check stop flag
            if self.stopped:
                return

            # If not stopping, grab new frame
            self.grabbed, self.frame = self.camStream.read()


    # Define frame read method
    def read_frame(self):

        # Declare frame for undistorted image
        newFrame = np.zeros(shape=(self.width, self.height, 3), dtype=np.uint8)

        try:

            # Grab new frame
            self.grabbed, frame = self.camStream.read()

            if not self.grabbed:
                return newFrame

            # Undistort image
            if self.undistort_img == True:
                h, w = frame.shape[:2]
                new_matrix, roi = cv.getOptimalNewCameraMatrix(self.cam_matrix,
                                                                self.distort_coeffs,
                                                                (w,h),1,(w,h))
                newFrame = cv.undistort(frame, self.cam_matrix,
                                        self.distort_coeffs, None,
                                        new_matrix)
                x,y,w,h = roi
                newFrame = newFrame[y:y+h,x:x+w]

            else:

                newFrame = frame

        except Exception as read_error:

            # Write error to log
            self.log_file.write("Error reading video:\n    type: {}\n    args: {}\n    {}\n".format(type(read_error), read_error.args, read_error))

        if self.cvs is not None:
            self.cvs.putFrame(cv.resize(newFrame, (self.width // self.streamRes, self.height // self.streamRes)))

        # Return the most recent frame
        return newFrame


    # Define video writing method
    def write_video(self, img):

        # Check if write is opened
        if self.camWriter.isOpened():

            # Write the image
            try:

                self.camWriter.write(img)
                return True

            except Exception as write_error:

                # Print exception info
                self.log_file.write("Error writing video:\n    type: {}\n    args: {}\n    {}\n".format(type(write_error), write_error.args, write_error))
                return False
        
        else:

            self.log_file.write("Video writer not opened!\n")
            return False


    # Define camera release method
    def release_cam(self):

        # Release the camera resource
        self.camStream.release()

        # Release video writer
        self.camWriter.release()

        # Close the log file
        self.log_file.write("Webcam closed. Video writer closed.\n")
        self.log_file.close()

    def use_libs(self, *libs):
        frame = self.read_frame()
        return (frame, *[lib.find_objects(frame, self.width, self.height, self.fov) for lib in libs])
    
    def _use_libs_update(self, callback, *libs):
        callback(*self.use_libs(*libs))

    def use_libs_async(self, *libs, callback = lambda _: None, name = "vision"):
        thread = Thread(target=self._use_libs_update, args=(callback, *libs), name=name)
        thread.daemon = True
        thread.start()
        return thread
