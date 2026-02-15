import socket
import cv2 as cv
import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge

rospy.init_node("py_connection")

publisher = rospy.Publisher('/detections', Image, queue_size=1)

bridge = CvBridge()

sock_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

input_image = cv.imread("input.jpg")

_, encoded = cv.imencode(".jpg", input_image, [cv.IMWRITE_JPEG_QUALITY, 80])

i_bytes  = encoded.tobytes()

sock_sender.sendto(i_bytes, ("192.168.31.200", 5000))

print("send")

sock_sender.close()


sock_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock_receiver.bind(("0.0.0.0", 5000))

rec_bytes, _ = sock_receiver.recvfrom(60000)
nparr = np.frombuffer(rec_bytes, np.uint8)
output_image = cv.imdecode(nparr, cv.IMREAD_COLOR)
cv.imwrite("output.jpg", output_image)

print("received")

sock_receiver.close()


imgmsg = bridge.cv2_to_imgmsg(output_image, 'bgr8')

while True:
  publisher.publish(imgmsg)
  print("toped")