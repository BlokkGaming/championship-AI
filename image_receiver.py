import socket
import numpy as np
import cv2 as cv

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind(("0.0.0.0", 5000))

while True:
    data, addr = sock.recvfrom(60000)
    nparr = np.frombuffer(data, np.uint8)
    image = cv.imdecode(nparr, cv.IMREAD_COLOR)
    cv.imwrite("output.jpg", image)
