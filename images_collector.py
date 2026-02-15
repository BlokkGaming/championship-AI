import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
import random
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
 
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

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

cooldown = 0
counter = 0
def image_callback(imgmsg):
    global cooldown
    global counter
    if cooldown < 7:
        cooldown += 1
    else: 
        cooldown = 0
        cv_img = bridge.imgmsg_to_cv2(imgmsg, 'bgr8')
        cv.imwrite(f"images/image_7_{counter}.jpg", cv_img)
        counter += 1

TARGETS = 200

def main():
    navigate_wait(x=0, y=0, z=1, frame_id='body',  auto_arm=True)
    for i in range(TARGETS):
        rx = random.randint(0, 9)
        ry = random.randint(0, 9)
        rz = 0.3 + random.random()
        print(rx, ry, rz)
        navigate_wait(x=rx, y=ry, z=rz, frame_id='aruco_map', speed=2)
    land()

rospy.Subscriber('/main_camera/image_raw_throttled', Image, image_callback)

main()