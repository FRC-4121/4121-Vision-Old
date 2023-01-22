from FRCVisionBase import VisionBase, math, cv

class ConeVisionLibrary(VisionBase):

    # Define class fields
    cone_values = {}


    # Define class initialization
    def __init__(self):
        pass

    # Locates the cubes and cones in the game (2023)
    # returns a tuple containing (cubes, cones)
    def find_objects(self, imgRaw, cameraWidth, cameraHeight, cameraFOV):
        
        # Read HSV values from dictionary and make tuples
        coneHSVMin = (int(VisionBase.config["CONES"]['HMIN']), int(VisionBase.config["CONES"]['SMIN']), int(VisionBase.config["CONES"]['VMIN']))
        coneHSVMax = (int(VisionBase.config["CONES"]['HMAX']), int(VisionBase.config["CONES"]['SMAX']), int(VisionBase.config["CONES"]['VMAX']))
        coneMinRadius = VisionBase.config["CONES"]['MINRADIUS']
        coneRadius    = VisionBase.config["CONES"]['RADIUS']
        
        # Initialize variables (Cone)
        conesFound = 0
        coneData = []
        coneTolerance = 0.1

        
        # Find contours in the mask and clean up the return style from OpenCV
        coneContours = self.process_image_contours(imgRaw, coneHSVMin, coneHSVMax, False, True)

        if len(coneContours) > 0:
            coneContours.sort(key=cv.contourArea, reverse=True)
            for contour in coneContours:
                ((x, y), radius) = cv.minEnclosingCircle(contour)
                
                if radius > float(coneMinRadius): # in pixel units
                    # Calculate ball metrics
                    inches_per_pixel = float(coneRadius) / radius # set up a general conversion factor
                    distanceToTargetPlane = inches_per_pixel * (cameraWidth / (2 * math.tan(math.radians(cameraFOV))))
                    offsetInInches = inches_per_pixel * (x - cameraWidth / 2)
                    angleToCone = math.degrees(math.atan((offsetInInches / distanceToTargetPlane)))
                    distanceToCone = math.cos(math.radians(angleToCone)) * distanceToTargetPlane
                    screenPercent = math.pi * radius * radius / (cameraWidth * cameraHeight)
                    coneOffset = -offsetInInches
                    
                    coneData.append({
                        'x': x,
                        'y': y,
                        'distance': distanceToCone,
                        'angle': angleToCone,
                        'offset': coneOffset,
                        'percent': screenPercent
                    })
        return coneData
