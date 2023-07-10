from socket import *

IP = '10.27.250.165'
PORT = 8000
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
