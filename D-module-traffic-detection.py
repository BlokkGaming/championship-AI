import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
import random
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import tkinter as tk

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

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

result_violation = False

def image_callback(imgmsg):
    global result_violation
    cv_img = bridge.imgmsg_to_cv2(imgmsg, 'bgr8')
    results = model(cv_img, verbose=False)
    result_violation = results[0]
    boxes = result_violation.boxes
    boxes_data = {}
    for i in range(len(boxes)): 
        boxes_data[result_violation.names[int(boxes.cls[i])]] = boxes.xywh[i].tolist(), # 'name': [x1, y1, x2, y2]
    result_violation = make_decision(boxes_data)
    
def make_decision(boxes_data):
    if 'green-light' in boxes_data:
        return False
    if 'red-light' not in boxes_data:
        return False
    else:
        try:
            rover_y = boxes_data['rover'][1]
            stopline_y = boxes_data['stop-line'][1]
            if rover_y > stopline_y:
                return True
            else:
                return False
        except Exception:
            pass

flag = True

def main():
    global flag
    navigate_wait(x=0, y=0, z=1, frame_id='body',  auto_arm=True)
    navigate_wait(x=2, y=2, z=1.5, frame_id='aruco_map')
    while not rospy.is_shutdown():
        if result_violation and flag:
            flag = False
            print("PRIJMITES K ABOCHINE")
            root = tk.Tk()  
            root.title("Message")
            root.geometry("450x250")  
            text = tk.Label(root, text="PRIJMITES K ABOCHINE", font=('Arial', 24, 'bold'), fg = 'red')
            text.pack(expand=True, fill='both') 
            root.after(3000, root.destroy)
            root.mainloop()
            navigate_wait(x=9, y=9, z=1.5, frame_id='aruco_map')
            land()
            



    
rospy.Subscriber('/main_camera/image_raw_throttled', Image, image_callback)

main()