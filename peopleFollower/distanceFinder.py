


REAL_DISTANCE = 23 # cm
REAL_WIDTH = 12 # cm
PIXEL_WIDTH_AT_REAL_DIST = 257 # pixels
FOCAL_LENGTH = (PIXEL_WIDTH_AT_REAL_DIST * REAL_DISTANCE) / REAL_WIDTH

##
## Finds distance to an object in centimeters
##
## :param      pixelWidth:  The pixel width
##
## :returns:   { real distance to an object }
##
def findCameraDistance(pixelWidth) :
	return (REAL_WIDTH * FOCAL_LENGTH) / pixelWidth


