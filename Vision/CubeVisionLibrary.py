from FRCVisionBase import *

class CubeVisionLibrary(VisionBase):

    # Define class initialization
    def __init__(self):
        pass

    # Locates the cubes and cones in the game (2023)
    # returns a tuple containing (cubes, cones)
    def find_objects(self, imgRaw, cameraWidth, cameraHeight, cameraFOV):
        
        # Read HSV values from dictionary and make tuples
        cubeHSVMin    = (int(VisionBase.config["CUBE"]['HMIN']), int(VisionBase.config["CUBE"]['SMIN']), int(VisionBase.config["CUBE"]['VMIN']))
        cubeHSVMax    = (int(VisionBase.config["CUBE"]['HMAX']), int(VisionBase.config["CUBE"]['SMAX']), int(VisionBase.config["CUBE"]['VMAX']))
        cubeMinArea   = float(VisionBase.config["CUBE"]['MINAREA'])
        cubeMinWidth  = float(VisionBase.config["CUBE"]['MINWIDTH'])
        cubeMinHeight = float(VisionBase.config["CUBE"]['MINHEIGHT'])
        cubeWidth     = float(VisionBase.config["CUBE"]['WIDTH'])
        cubeHeight    = float(VisionBase.config["CUBE"]['HEIGHT'])
        
        # Initialize variables (Cube)
        cubeData = []
        cubeTolerance = 0.1

        
        # Find contours in the mask and clean up the return style from OpenCV
        cubeContours = self.process_image_contours(imgRaw, cubeHSVMin, cubeHSVMax, False, True)

        if len(cubeContours) > 0:

            cubeContours.sort(key=cv.contourArea, reverse=True)
            for contour in cubeContours:
                x, y, w, h = cv.boundingRect(contour)
                
                if w < cubeMinWidth or h < cubeMinHeight or w * h < cubeMinArea: # in pixel units
                    break
                # Calculate cube metrics
                inches_per_pixel = float(cubeHeight) / h # set up a general conversion factor
                distanceToTargetPlane = inches_per_pixel * (cameraWidth / (2 * math.tan(math.radians(cameraFOV))))
                offsetInInches = inches_per_pixel * (x - cameraWidth / 2)
                angleToCube = math.degrees(math.atan((offsetInInches / distanceToTargetPlane)))
                distanceToCube = math.cos(math.radians(angleToCube)) * distanceToTargetPlane
                screenPercent = w * h / (cameraWidth * cameraHeight)
                cubeOffset = -offsetInInches
                
                cubeData.append(FoundObject("CUBE", x, y,
                    w=w,
                    h=h,
                    distance=distanceToCube,
                    angle=angleToCube,
                    offset=cubeOffset,
                    percent=screenPercent
                ))
        
        return cubeData
