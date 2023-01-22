from FRCVisionBase import *

class TapeRectVisionLibrary(VisionBase):

    # Define class fields
    tape_values = {}


    # Define class initialization
    def __init__(self):
        pass

    # Locates the cubes and cones in the game (2023)
    # returns a tuple containing (cubes, cones)
    def find_objects(self, imgRaw, cameraWidth, cameraHeight, cameraFOV):
        
        config = VisionBase.config["TAPE"]
        # Read configuration values from dictionary and make tuples
        HSVMin    = (int(config['HMIN']), int(config['SMIN']), int(config['VMIN']))
        HSVMax    = (int(config['HMAX']), int(config['SMAX']), int(config['VMAX']))
        minArea   = float(config['MINAREA'])
        tolerance = float(config['TOLERANCE'])
        width     = float(config['WIDTH'])
        height    = float(config['HEIGHT'])
        aspect    = height / width
        
        # Initialize variables (Cone)
        data = []
        

        
        # Find contours in the mask and clean up the return style from OpenCV
        contours = self.process_image_contours(imgRaw, HSVMin, HSVMax, False, False)
        if len(contours) > 0:

            contours.sort(key=cv.contourArea, reverse=True)
            for contour in contours:
                x, y, w, h = cv.boundingRect(contour)
                
                if w * h < minArea: # in pixel units
                    break
                
                if abs(h / w / aspect - 1.0) > tolerance:
                    continue

                # Calculate metrics
                inches_per_pixel = float(height) / h # set up a general conversion factor
                distanceToTargetPlane = inches_per_pixel * (cameraWidth / (2 * math.tan(math.radians(cameraFOV))))
                offsetInInches = inches_per_pixel * ((x + (w / 2)) - (cameraWidth / 2))
                angleToObject = -1 * math.degrees(math.atan((offsetInInches / distanceToTargetPlane)))
                distanceToObject = math.cos(math.radians(angleToObject)) * distanceToTargetPlane
                screenPercent = w * h / (cameraWidth * cameraHeight)
                offset = -offsetInInches
                
                data.append(FoundObject("CONE", x, y,
                    w=w,
                    h=h,
                    distance=distanceToObject,
                    angle=angleToObject,
                    offset=offset,
                    percent=screenPercent
                ))
        
        return data