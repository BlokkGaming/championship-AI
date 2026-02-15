import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

message = input()
m_bytes = message.encode('utf-8')

sock.sendto(m_bytes, ("192.168.31.182", 5000))

print("sent")

sock.close()

