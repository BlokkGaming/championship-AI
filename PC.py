import socket
import cv2 as cv
import numpy as np
from ultralytics import YOLO

model = YOLO("best.pt", verbose=False)

sock_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock_receiver.bind(("0.0.0.0", 5000))

rec_bytes, _ = sock_receiver.recvfrom(60000)
nparr = np.frombuffer(rec_bytes, np.uint8)
input_image = cv.imdecode(nparr, cv.IMREAD_COLOR)

print("received")

sock_receiver.close()


output_image = model(input_image)[0].plot()


sock_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

_, encoded = cv.imencode(".jpg", output_image, [cv.IMWRITE_JPEG_QUALITY, 80])

i_bytes  = encoded.tobytes()

sock_sender.sendto(i_bytes, ("192.168.31.197", 5000))

print("send")

sock_sender.close()
