#!/usr/bin/env python3

##################################################################################
# The MIT License (MIT)
# 
# Copyright (c) 2015 MicaSense, Inc.
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
##################################################################################

import requests
import matplotlib.pyplot as plt
import cv2
import tifffile as tiff
import io
from PIL import Image
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

#Declare the IP of the sensor
ip="http://192.168.10.254"
rosRate=1
#Read the GPS data from the camera
#gps_data = requests.get('http://192.168.10.254/gps')
#print ("Raw data : " + str(gps_data.text))
#print ("Has 3d fix: " + str(gps_data.json()['fix3d']))
#Post a message to the camera commanding a capture, block until complete
class micasense(object):
	def __init__ (self):	
		self.pub_channel1 = rospy.Publisher('/micasense/channel1/compressed',CompressedImage,queue_size=5)
		self.pub_channel2 = rospy.Publisher('/micasense/channel2/compressed',CompressedImage,queue_size=5)
		self.pub_channel3 = rospy.Publisher('/micasense/channel3/compressed',CompressedImage,queue_size=5)
		self.pub_channel4 = rospy.Publisher('/micasense/channel4/compressed',CompressedImage,queue_size=5)
		self.pub_channel5 = rospy.Publisher('/micasense/channel5/compressed',CompressedImage,queue_size=5)

	def pub_ros(self, index, image):
		if (index==0):
			publisher=self.pub_channel1
		elif(index==1):
			publisher=self.pub_channel2
		elif(index==2):
			publisher=self.pub_channel3
		elif(index==3):
			publisher=self.pub_channel4
		elif(index==4):
			publisher=self.pub_channel5
		
		image_msg=CompressedImage()
		image_msg.header.stamp=rospy.Time.now()
		image_msg.format="jpeg"
		image_msg.data=np.array(cv2.imencode('.jpg',image)[1]).tobytes()

		#Publicador de la imagen comprimida
		publisher.publish(image_msg)



	def pub_image(self,event=None):
		self.index=[]
		capture_params = { 'store_capture' : False, 'block' : True }
		capture_data = requests.post("http://192.168.10.254/capture", json=capture_params)
		#print (capture_data.json())

		info=capture_data.json()
		self.index.append(info["raw_cache_path"]["1"])
		self.index.append(info["raw_cache_path"]["2"])
		self.index.append(info["raw_cache_path"]["3"])
		self.index.append(info["raw_cache_path"]["4"])
		self.index.append(info["raw_cache_path"]["5"])

		#Post a message to download the cached version of the image
		
		for i in range(0, len(self.index)):
			dir=ip+self.index[i]
			cache_im = requests.get(dir)
			img = Image.open(io.BytesIO(cache_im.content))
			img_resized = img.resize((1024, 768))
			image_array = np.array(img_resized, dtype='uint16')
			cvuint8 = cv2.convertScaleAbs(image_array,alpha=(255.0/65535.0))
			self.pub_ros(i,cvuint8)

		
		# resize image
		#dim = (640, 480)
		#resized = cv2.resize(cvuint8, dim, interpolation = cv2.INTER_AREA)

		#window_name = 'image'
		#cv2.imshow(window_name,cvuint8)
		#cv2.waitKey(0)
		#cv2.destroyAllWindows()

		#Para publicar en el topic de CompressedImage
		#image_msg=CompressedImage()
		#image_msg.header.stamp=rospy.Time.now()
		#image_msg.format="jpeg"
		#image_msg.data=np.array(cv2.imencode('.jpg',cvuint8)[1]).tobytes()

		#Publicador de la imagen comprimida
		#self.pub_channel1.publish(image_msg)


	def start(self):
		rospy.Timer(rospy.Duration(1.0/rosRate), self.pub_image)
		rospy.spin()


if __name__ == '__main__':
	# starts the node
	rospy.init_node('micasense')
	micasense1=micasense()
	try:
			micasense1.start()
	except rospy.ROSInterruptException:
			pass