 # Import socket module 
import socket             
 
# Create a socket object 
s = socket.socket()         
 
# Define the port on which you want to connect 
port = 14475
              
 
# connect to the server on local computer 
s.connect(('18.189.106.45', port)) 

s.send(b'0.0,0.0,0.0,0.0') 
 
# receive data from the server and decoding to get the string.

data = s.recv(1024)
print(data)
print("+++")
print(data.decode())

# close the connection 
s.close()