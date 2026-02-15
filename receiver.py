import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind(("0.0.0.0", 5000))
while True:
    data, addr = sock.recvfrom(1024)
    message = data.decode("utf-8")
    print(message)
    if message == "the end":
        sock.close
        break
