# CENTROID TRACKER BASED ON:
# https://www.pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/



from scipy.spatial import distance as dist
from collections import OrderedDict
from utils import rectArea
import numpy as np

# This tracker will remember objects that were recognized by color tresholding
class CentroidTracker() :
	def __init__(self, maxDisappeared=50) :
		self.nextObjectId = 0				# counter for assigned unique ID's
		self.objects = OrderedDict()		# A dictionary with ID as a key and centroid (x,y) as value
		self.disappeared = OrderedDict()	# Keeps track of consecutive frames frames number (value) that object with ID as key was missing
		self.maxDisappeared = maxDisappeared # maximum number of frames that an object is out of the camera sight for to consider it missing
	def registerObject(self, centroid) :
		# adds an object to the dictionary
		self.objects[self.nextObjectId] = centroid
		self.disappeared[self.nextObjectId] = 0 #since we found the object it is not disappeared so we set count to 0
		self.nextObjectId += 1 # increment objectID count

	def deregisterObject(self, objectID) :
		del self.objects[objectID]
		del self.disappeared[objectID]

	def update(self, rects) :
		if len(rects) == 0:
			for objectID in list(self.disappeared.keys()) :
				self.disappeared[objectID] += 1

				if self.disappeared[objectID] > self.maxDisappeared:
					self.deregisterObject(objectID)
				
			return self.objects
		# initialize the container for input centroids
		inputCentroids = np.zeros((len(rects),3), dtype="int")

		for (i, (startX, startY, endX, endY)) in enumerate(rects) :
			centerX = int((startX + endX) / 2.0)
			centerY = int((startY + endY) / 2.0)

			area = rectArea((endX - startX), (endY - startY))
			inputCentroids[i] = (centerX, centerY, area)
			pass

		if len(self.objects) == 0:
			# in case there is no objects registered yet, register all of them 
			for i in range(0, len(inputCentroids)):
				self.registerObject(inputCentroids[i])
				pass
		# otherwise che
		else:
			objectIDs = list(self.objects.keys())
			objectCentroids = list(self.objects.values())

			#find distances between each pair of existing objects centroids and centroids from current frame, we need to match existing centroid to centroid from current frame
			distance = dist.cdist(np.array(objectCentroids), inputCentroids)

			# in order to perform this matching we must (1) find the
			# smallest value in each row and then (2) sort the row
			# indexes based on their minimum values so that the row
			# with the smallest value is at the *front* of the index
			# list
			rows = distance.min(axis=1).argsort()
			# next, we perform a similar process on the columns by
			# finding the smallest value in each column and then
			# sorting using the previously computed row index list
			cols = distance.argmin(axis=1)[rows]

			usedRows = set()
			usedCols = set()

			# loop over the combination of the (row, column) index
			# tuples
			for (row, col) in zip(rows, cols):
				# if we have already examined either the row or
				# column value before, ignore it
				# val
				if row in usedRows or col in usedCols:
					continue
				# otherwise, grab the object ID for the current row,
				# set its new centroid, and reset the disappeared
				# counter
				objectID = objectIDs[row]
				self.objects[objectID] = inputCentroids[col]
				self.disappeared[objectID] = 0
				# indicate that we have examined each of the row and
				# column indexes, respectively
				usedRows.add(row)
				usedCols.add(col)
				# compute both the row and column index we have NOT yet
			# examined
			unusedRows = set(range(0, distance.shape[0])).difference(usedRows)
			unusedCols = set(range(0, distance.shape[1])).difference(usedCols)

			# in the event that the number of object centroids is
			# equal or greater than the number of input centroids
			# we need to check and see if some of these objects have
			# potentially disappeared
			if distance.shape[0] >= distance.shape[1]:
				# loop over the unused row indexes
				for row in unusedRows:
					# grab the object ID for the corresponding row
					# index and increment the disappeared counter
					objectID = objectIDs[row]
					self.disappeared[objectID] += 1
					# check to see if the number of consecutive
					# frames the object has been marked "disappeared"
					# for warrants deregistering the object
					if self.disappeared[objectID] > self.maxDisappeared:
						self.deregisterObject(objectID)
			else:
				for col in unusedCols:
					self.registerObject(inputCentroids[col])
			return self.objects
