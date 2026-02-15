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
from aruco_pose.msg import MarkerArray 

os.chdir("/home/clover/Desktop/ChampionshipAI/")

model = YOLO('best.pt')

rospy.init_node('flight')

latest_markers = MarkerArray()
rover_coords = None

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

def markers_callback(data):
    global latest_markers 
    latest_markers = data 

def image_callback(msg):
    global rover_coords
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    results = model(cv_image, verbose=False)

    if len(results) > 0:
        result = results[0]
        cv_result = result.plot()
        boxes = result.boxes
        rover_pixel_x = None
        rover_pixel_y = None
        for i in range(len(boxes)):
            xywh = boxes.xywh[i]
            rover_pixel_x = xywh[0].item()
            rover_pixel_y = xywh[1].item()

        if rover_pixel_x is not None and rover_pixel_y is not None:

            imgmsg_res = bridge.cv2_to_imgmsg(cv_result, "bgr8")
            result_pub.publish(imgmsg_res)

            img_height = cv_image.shape[0]
            img_width = cv_image.shape[1]
            img_center = (img_width/2, img_height/2)
    
            min_pixel_aruco_to_center_distance = None

            chosen_aruco_coords = None

            for marker in latest_markers.markers:
                marker_real_coords = (marker.pose.position.x, marker.pose.position.y)
                marker_pixel_coords = get_aruco_center([marker.c1, marker.c2, marker.c3, marker.c4])

                pixel_distance = get_distance(marker_pixel_coords, img_center)

                if (min_pixel_aruco_to_center_distance is None) or (pixel_distance < min_pixel_aruco_to_center_distance):
                    min_pixel_aruco_to_center_distance = pixel_distance
                    chosen_aruco_coords = marker_real_coords

            if chosen_aruco_coords is not None:
    
                clover_coords = (get_telemetry(frame_id='aruco_map').x, get_telemetry(frame_id='aruco_map').y)

                real_distance_to_aruco = pythagorean_theorem(chosen_aruco_coords[0], chosen_aruco_coords[1])

                similarity_coefficient = real_distance_to_aruco / min_pixel_aruco_to_center_distance


                pixel_x_to_rover_distance = rover_pixel_x - img_center[0]
                pixel_y_to_rover_distance = rover_pixel_y - img_center[1]

                real_x_to_rover_distance = pixel_x_to_rover_distance * similarity_coefficient
                real_y_to_rover_distance = pixel_y_to_rover_distance * similarity_coefficient


                rover_coords = (clover_coords[0] + real_x_to_rover_distance, clover_coords[1] + real_y_to_rover_distance)

    else:
        rover_coords = None

    print(rover_coords)

def get_aruco_center(corners):
    sum_x = 0
    sum_y = 0

    for corner in corners:
        sum_x += corner.x
        sum_y += corner.y
    
    return (sum_x/4, sum_y/4)

def get_distance(coords1, coords2):
    catheter1 = coords1[0] - coords2[0]
    catheter2 = coords1[1] - coords2[1]

    return pythagorean_theorem(catheter1, catheter2)

def pythagorean_theorem(a, b):
    c = math.sqrt(a**2 + b**2)
    return c


image_sub = rospy.Subscriber("/main_camera/image_raw_throttled", Image, image_callback)
result_pub = rospy.Publisher("/detections", Image, queue_size=1)
markers_sub = rospy.Subscriber('/aruco_detect/markers', MarkerArray, markers_callback) 

def main():
    global rover_coords
    navigate_wait(z=1, frame_id='body', auto_arm=True)
    navigate_wait(x=4, y=4, z=1, frame_id='aruco_map', speed=0.5)

    while not rospy.is_shutdown():
        # if rover_coords is not None:
        #     navigate(x=rover_coords[0], y=rover_coords[1], z = 1.5, frame_id='aruco_map', speed=1)
        # else:
            pass
main()