from FRCVisionBase import VisionBase, math, cv

class CubeVisionLibrary(VisionBase):

    # Define class fields
    cube_values = {}


    # Define class initialization
    def __init__(self, visionfile):
        
        #Read in vision settings file
        super(visionfile, {"CUBES": self.cube_values})

    # Locates the cubes and cones in the game (2023)
    # returns a tuple containing (cubes, cones)
    def find_objects(self, imgRaw, cameraWidth, cameraHeight, cameraFOV):
        
        # Read HSV values from dictionary and make tuples
        cubeHSVMin = (int(self.cube_values['HMIN']), int(self.cube_values['SMIN']), int(self.cube_values['VMIN']))
        cubeHSVMax = (int(self.cube_values['HMAX']), int(self.cube_values['SMAX']), int(self.cube_values['VMAX']))
        cubeMinRadius = self.cube_values['MINRADIUS']
        cubeRadius    = self.cube_values['RADIUS']
        
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
