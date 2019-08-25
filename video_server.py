import socket, os, signal, psutil
from subprocess import Popen, PIPE, DEVNULL

def find_free_port():
    s = socket.socket()
    # Bind to a free port provided by the host.
    s.bind(('', 0))
    return s.getsockname()[1]

HOST = '91.230.195.104'
PORT = 65440

lastProcess = None
currentPID = os.getpid()
      
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
   
   s.bind((HOST, PORT))
   s.listen()
   
   while True:
       try:
           print('Waiting for new Connection..')
           
           conn, addr = s.accept()
           
           if lastProcess != None:
               current_process = psutil.Process(lastProcess.pid)
               children = current_process.children(recursive=True)
               for child in children:
                   if child.pid != currentPID:
                       os.kill(child.pid, signal.SIGKILL)
           
           print('\nConnected to: '+str(addr))
        
           port_sender = find_free_port()
           port_videolink = None
           
           while True:
               port_videolink = find_free_port()
               if port_sender != port_videolink:
                   break
        
           with conn:
                data = conn.recv(1024)
                print('DRONE ID: '+str(data.decode()))
                if data:
                    conn.sendall(bytes(str(port_sender)+':'+str(port_videolink), 'utf-8'))
                    lastProcess = Popen("/usr/bin/python3 /opt/video_server/video_receiver.py --port_videolink="+str(port_videolink)+
                                                       " --port_sender="+str(port_sender), shell=True)
                    print("Videofeed port: "+str(port_videolink)+"\n")
       except Exception as e:
             print(str(e))
