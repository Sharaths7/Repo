
import cv2
import numpy as np
# Create a VideoCapture object and read from input file


def ConnectToDrone(rtmp_address):
	cap = cv2.VideoCapture(rtmp_address)
	# Check if camera opened successfully
	if (cap.isOpened()== False):
		print("Error opening video file")

	# Read until video is completed
	while(cap.isOpened()):
		
	# Capture frame-by-frame
		ret, frame = cap.read()
		if ret == True:
		# Display the resulting frame
			(flag, encodeImage) = cv2.imencode(".jpg", frame)
			cv2.imshow('Frame', frame)

			if cv2.waitKey(25) & 0xFF == ord('q'):
				break

	# Break the loop
		else:
			break

	# When everything done, release
	# the video capture object
	cap.release

	# Closes all the frames
	cv2.destroyAllWindows()


ConnectToDrone('rtmp://localhost:1935/live')






















































































































































































































































































































































































































































