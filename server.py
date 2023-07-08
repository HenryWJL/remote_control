from socket import *

IP = '10.42.0.1'
PORT = 9600
BUFLEN = 512
listenSocket = socket(AF_INET, SOCK_STREAM)
 
listenSocket.bind((IP, PORT))
 
listenSocket.listen(1)
print('The server has already started')
 
dataSocket, address = listenSocket.accept()
print('Connected to ', address)
 
while True:
    data = dataSocket.recv(BUFLEN)
    if not data:
        break
    info = data.decode()
    dataSocket.send(f'The server received {info.encode()}')
 
dataSocket.close()
listenSocket.close()
