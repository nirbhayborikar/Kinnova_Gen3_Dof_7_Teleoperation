import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 5005))
print("Listening on UDP port 5005... Waiting for phone...")

while True:
    data, addr = sock.recvfrom(1024)
    print(f"Data arrived from {addr}!")