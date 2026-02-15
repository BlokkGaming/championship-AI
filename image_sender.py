import socket
import cv2 as cv

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

cv_image = cv.imread("input.jpg")

_, encoded = cv.imencode(".jpg", cv_image, [cv.IMWRITE_JPEG_QUALITY, 80])

i_bytes = encoded.tobytes()

sock.sendto(i_bytes, ("192.168.31.200", 5000))

print("sent")

sock.close()

