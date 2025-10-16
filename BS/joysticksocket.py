import socket

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_ip = "192.168.5.2" # Sending to Pi from Mac
server_port = 7777
s.bind((server_ip, server_port))
