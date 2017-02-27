from math import *
rad2deg = 180/pi

#constants
KNOWN_HEIGHT = 5.0 #inches
KNOWN_WIDTH = 10.25 #inches
CAMERA_OFFSET = -20.0 #inches offset from center of robot, in Y direction
#calibration factors
KNOWN_DISTANCE = 36.0 #inches
FOCAL_POINT = 248.0 * KNOWN_DISTANCE / KNOWN_HEIGHT
HFOV = 0.449422282 #calibrated last year, same camera
HRES = 240.0 #screen width in pixels

def getDistance(measuredHeight):
    #calculates distance based on calibration factor
    return KNOWN_HEIGHT * FOCAL_POINT / measuredHeight

def getAngle(d1, d2, a):
    #add pi/2 to get angle of the normal
    # this is just cosine law + viewing angle + 90 degrees
    return a + acos( (d2**2 - d1**2 - KNOWN_WIDTH**2) / (-2*d1*KNOWN_WIDTH) ) + (pi/2) 

def getCenter(d1, d2, x):
    a = atan( 2 * x * tan(HFOV) / HRES )
    d = (d1+d2)/2
    x = d * cos(a)
    y = d * sin(a) + CAMERA_OFFSET
    return (x, y) #returns coordinate of peg in inches

d1 = getDistance(100) #distance to left target
d2 = getDistance(100) #distance to right target

offset = 65 #pixels from center of camera to left target
viewD = (d1+d2)/2 #distance to peg, average of distances to targets
viewA = atan( 2 * offset * tan(HFOV) / HRES ) #viewing angle of the left target
centerX = 50 #pixel offset of peg

print 'Target normal (deg):', getAngle(d1, d2, viewA) * rad2deg
print 'Target coordinate (in):', getCenter(d1, d2, centerX)
