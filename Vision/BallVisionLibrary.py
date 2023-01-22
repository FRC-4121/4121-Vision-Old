from FRCVisionBase import VisionBase, math, cv

class BallVisionLibrary(VisionBase):

    # Define class initialization
    def __init__(self, color):
        self.color = color

    # Locates the cubes and cones in the game (2023)
    # returns a tuple containing (cubes, cones)
    def find_objects(self, imgRaw, cameraWidth, cameraHeight, cameraFOV):
        
        config = VisionBase.config["BALL" + str(self.color)]

        # Define variables
        hMin = config["HMIN"]
        hMax = config["HMAX"]
        sMin = config["SMIN"]
        sMax = config["SMAX"]
        vMin = config["VMIN"]
        vMax = config["VMAX"]

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
                minRadius = config["MINRADIUS"]
                ballRadius = config["RADIUS"]

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

                    #Add dictionary to return list
                    ballData.append({
                        "x": x,
                        "y": y,
                        "radius": radius,
                        "distance": distanceToBall,
                        "angle": angleToBall,
                        "offset": ballOffset,
                        "percent": screenPercent
                    })

                    #Increment ball count
                    ballsFound += 1
        
                else:
                    #No more contours meet criteria so break loop
                    break

        return ballsFound, ballData

class BallVisionOnlyLibrary(VisionBase):

    # Define class fields
    ball_values = []


    # Define class initialization
    def __init__(self, visionfile, color):
        # Extend ball_values
        if color >= BallVisionLibrary.ball_values:
            BallVisionLibrary.ball_values.extend([{}] * (color - len(BallVisionLibrary.ball_values) + 1))
        #Read in vision settings file
        super(visionfile, {"BALL" + BallVisionLibrary.ball_values[color]: self.cube_values})
        self.color = color

    # Locates the cubes and cones in the game (2023)
    # returns a tuple containing (cubes, cones)
    def find_objects(self, imgRaw, cameraWidth, cameraHeight, cameraFOV):
        config = self.ball_values[self.color]

        # Define variables
        hMin = config["HMIN"]
        hMax = config["HMAX"]
        sMin = config["SMIN"]
        sMax = config["SMAX"]
        vMin = config["VMIN"]
        vMax = config["VMAX"]

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
                    minRadius = config["MINRADIUS"]
                    ballRadius = config["RADIUS"]

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

                        #Add dictionary to return list
                        ballData.append({
                            "x": x,
                            "y": y,
                            "radius": radius,
                            "distance": distanceToBall,
                            "angle": angleToBall,
                            "offset": ballOffset,
                            "percent": screenPercent
                        })

                        #Increment ball count
                        ballsFound += 1
            
                    else:
                        #No more contours meet criteria so break loop
                        break

        return ballsFound, ballData

