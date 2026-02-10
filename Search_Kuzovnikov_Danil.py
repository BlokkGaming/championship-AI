import rospy 
from clover import srv 
from clover.srv import SetLEDEffect 
from std_srvs.srv import Trigger 
from sensor_msgs.msg import Image 
import math 
from aruco_pose.msg import MarkerArray 
import numpy as np 
import cv2 as cv 
from cv_bridge import CvBridge 
import imutils 

bridge = CvBridge() 

rospy.init_node('flight') 

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry) 
navigate = rospy.ServiceProxy('navigate', srv.Navigate) 
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect) 
land = rospy.ServiceProxy('land', Trigger) 

latest_markers = MarkerArray() 

parks = [0, 10, 20] 
free_arucos = set() 

flag_led = False 

colors_led = { 
    "Electric Lime": [204, 255, 0],
    "Orchid":        [218, 112, 214],
    "Сrimson":       [220, 20,  60],
    "blue":          [0, 0, 255],
}

colors_detect_hsv = { 
    'blue': [np.array([110,200,120]), np.array([130,255,255])],
    'red': [np.array([0,100,60]), np.array([10,255,255])],
    'yellow': [np.array([20,200,120]), np.array([40,255,255])],
    'green': [np.array([50,200,120]), np.array([70,255,255])]
}

def navigate_wait(x = 0 , y = 0, z = 1.5, speed = 0.5, frame_id = "aruco_map", auto_arm = False, tolerance = 0.2):
    navigate(x=x, y=y, z=z, speed=speed, frame_id = frame_id, auto_arm = auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id = "navigate_target")
        telem_auto = get_telemetry()
        
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            rospy.sleep(3)
            break

        rospy.sleep(0.2)

def set_led(color):
    rgb = colors_led.get(color)
    set_effect(r = rgb[0], g = rgb[1], b = rgb[2], effect = 'fill')

def markers_callback(data):
    global latest_markers 
    latest_markers = data 

def image_callback(data):
    global latest_markers 

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8') 
    img_hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV) 
    for name, color in colors_detect_hsv.items(): 
        contours = get_contours(img_hsv, color) 
        for contour in contours:
            area = cv.contourArea(contour) 
            if area > 100: 
                cv.drawContours(cv_image, [contour], -1, (0,0,0), 5) 
                cv.drawContours(cv_image, [contour], -1, (255,255,255), 2) 

    led = False 
    for marker in latest_markers.markers:
        id = marker.id 
        free_arucos.add(id) 
        if id in parks: 
            led = True
            c1, c2, c3, c4 = marker.c1, marker.c2, marker.c3, marker.c4 

            x_stp = int(c4.x - c1.x) 
            y_stp = int(c1.y - c2.y)

            cv.rectangle(cv_image, (int(c1.x) - x_stp, int(c1.y) + 2 * y_stp), (int(c3.x) + x_stp, int(c3.y) - 2 * y_stp), (0,255,0), 6)

    if flag_led: 
        if led: 
            set_led("Сrimson") 
        else: 
            set_led("Orchid") 

    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

def get_contours(frame, color):
    mask = cv.inRange(frame, color[0], color[1]) 
    contours = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE) 
    contours = imutils.grab_contours(contours)
    return contours 

def get_coords_by_aruco(aruco):
    return aruco % 10, 9 - aruco // 10

def main():
    global flag_led 
    flag_led = False 
    set_led("Electric Lime") 
    navigate_wait(z=1.5, speed = 1, frame_id="body", auto_arm=True) 
    flag_led = True
    navigate_wait(z=1.5, y = 10, x = 0, speed = 1, frame_id="aruco_map", auto_arm=True)
    navigate_wait(z=1.5, y = 0, x = 0, speed = 1, frame_id="aruco_map", auto_arm=True)
    flag_led = False 
    set_led("blue") 
    land() 
    
    file = open("Search_Kuzovnikov_Danil.txt", "w") 
    for p in parks: 
        if p in free_arucos:
            place = "free"
        else:
            place = "busy" 
        x, y = get_coords_by_aruco(p) 
        file.write(f"Mesto {p//10+1}: {x} {y}, {place}\n") 

    file.close() 

image_sub = rospy.Subscriber('/main_camera/image_raw', Image, image_callback) 
image_pub = rospy.Publisher('/Kuzovnikov_Danil_debug', Image, queue_size=1) 
markers_sub = rospy.Subscriber('/aruco_detect/markers', MarkerArray, markers_callback) 

main() 
  