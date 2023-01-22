from FRCVisionBase import VisionBase, math, cv

class CubeVisionLibrary(VisionBase):

    # Define class initialization
    def __init__(self):
        pass

    # Locates the cubes and cones in the game (2023)
    # returns a tuple containing (cubes, cones)
    def find_objects(self, imgRaw, cameraWidth, cameraHeight, cameraFOV):
        
        # Read HSV values from dictionary and make tuples
        cubeHSVMin = (int(VisionBase.config["CUBES"]['HMIN']), int(VisionBase.config["CUBES"]['SMIN']), int(VisionBase.config["CUBES"]['VMIN']))
        cubeHSVMax = (int(VisionBase.config["CUBES"]['HMAX']), int(VisionBase.config["CUBES"]['SMAX']), int(VisionBase.config["CUBES"]['VMAX']))
        cubeMinRadius = VisionBase.config["CUBES"]['MINRADIUS']
        cubeRadius    = VisionBase.config["CUBES"]['RADIUS']
        
        # Initialize variables (Cube)
        cubesFound = 0
        cubeData = []
        cubeTolerance = 0.1

        
        # Find contours in the mask and clean up the return style from OpenCV
        cubeContours = self.process_image_contours(imgRaw, cubeHSVMin, cubeHSVMax, False, True)

        if len(cubeContours) > 0:
            cubeContours.sort(key=cv.contourArea, reverse=True)
            for contour in cubeContours:
                ((x, y), radius) = cv.minEnclosingCircle(contour)
                
                if radius > float(cubeMinRadius): # in pixel units
                    # Calculate ball metrics
                    inches_per_pixel = float(cubeRadius) / radius # set up a general conversion factor
                    distanceToTargetPlane = inches_per_pixel * (cameraWidth / (2 * math.tan(math.radians(cameraFOV))))
                    offsetInInches = inches_per_pixel * (x - cameraWidth / 2)
                    angleToCube = math.degrees(math.atan((offsetInInches / distanceToTargetPlane)))
                    distanceToCube = math.cos(math.radians(angleToCube)) * distanceToTargetPlane
                    screenPercent = math.pi * radius * radius / (cameraWidth * cameraHeight)
                    cubeOffset = -offsetInInches
                    
                    cubeData.append({
                        'x': x,
                        'y': y,
                        'distance': distanceToCube,
                        'angle': angleToCube,
                        'offset': cubeOffset,
                        'percent': screenPercent
                    })
        
        return cubeData
