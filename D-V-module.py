import cv2
from clover import srv
from std_srvs.srv import Trigger
import rospy
import math
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import os
from ultralytics import YOLO

model = YOLO('best.pt')

rospy.init_node('flight')

bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='body', auto_arm=False, tolerance = 0.2):
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        telem_auto = get_telemetry()

        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            rospy.sleep(3)
            break
            
        rospy.sleep(0.2)

def image_callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    results = model(cv_image, verbose=False)

    if len(results) > 0:
        result = results[0]
        cv_result = result.plot()
        boxes = result.boxes

        for i in range(len(boxes)):
            name = result.names[int(boxes.cls[i])]
            xywh = boxes.xywh[i]
            conf = boxes.conf[i]

            print("Found:")
            print("name: " + str(name))
            print("xywh: " + str(xywh))
            print("conf: " + str(conf))
            print()

        imgmsg_res = bridge.cv2_to_imgmsg(cv_result, "bgr8")
        result_pub.publish(imgmsg_res)

    else:
        print("not found\n")


image_sub = rospy.Subscriber("/main_camera/image_raw_throttled", Image, image_callback)
result_pub = rospy.Publisher("/detections", Image, queue_size=1)

def main():
    navigate_wait(z=1, frame_id='body', auto_arm=True)
    navigate_wait(x=1, y=4, z=1.5, frame_id='aruco_map', speed=0.5)
    navigate_wait(x=4, y=1, z=1.5, frame_id='aruco_map', speed=0.5)
    navigate_wait(x=2, y=2, z=1.5, frame_id='aruco_map', speed=0.5)
    navigate_wait(x=0, y=0, z=1.5, frame_id='aruco_map', speed=0.5)
    land()

main()