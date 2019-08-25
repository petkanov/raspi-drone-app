import socket, os, signal
import threading
import time
import queue 
import ProtoData_pb2 as proto
from drone import Drone 
from subprocess import Popen

class DataReceiver (threading.Thread):
   def __init__(self, socket, queue, drone):
      threading.Thread.__init__(self)
      self.daemon = True
      self.socket = socket
      self.queue = queue
      self.drone = drone
      
   def run(self):
      while(True):
          try:
              data = self.socket.recv(1024)
              if not data or data.decode() == "quit":
                  self.socket.close()
                  break
              #queue.put(data)
              
              command = proto.Command()
              command.ParseFromString(data)

              self.drone.executeCommand(command)
              
          except Exception as e:
              print("Receiver killed: "+str(e))
              break


class Consumer (threading.Thread):
   def __init__(self, queue, drone):
      threading.Thread.__init__(self)
      self.daemon = True
      self.queue = queue
      self.drone = drone
      
   def run(self):
      while(True):
          try:
              data = self.queue.get()

              command = proto.Command()
              command.ParseFromString(data)

              print(str(command))
              drone.executeCommand(command)
        
              self.queue.task_done()
          except:
              print("Consumer killed")
              break
	

#CONTROL_HOST = '192.168.170.48'
CONTROL_HOST = '91.230.195.104'
VIDEO_SERVER_PORT = 65440
DRONE_CLOUD_SERVER_PORT = 1314

if __name__ == '__main__':
    
    videoServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    videoServerSocket.connect((CONTROL_HOST, VIDEO_SERVER_PORT))
    videoServerSocket.sendall(b'Peter Drone 1')
    
    data = videoServerSocket.recv(1024).decode()
    
    streamer_port, video_feed_port = data.split(':')
    
    print("Video Feed Port: "+str(video_feed_port))
    
    videoStreamerProc = Popen('python3 video_streamer.py --port='+str(streamer_port), shell=True)

    controlServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while True:
        try:
            controlServerSocket.connect((CONTROL_HOST, DRONE_CLOUD_SERVER_PORT))
            break;
        except Exception as e:
            print('Could not connect to remote Server: '+str(e))
            time.sleep(2)

    queue = queue.Queue(maxsize=1024)
    drone = Drone("192.168.170.57", 14550, video_feed_port)
    serverMessageReceiver = DataReceiver(controlServerSocket, queue, drone)
    serverMessageReceiver.start()

    #serverMessageConsumer = Consumer(queue, drone)
    #serverMessageConsumer.start()
    
    try:
        while(True):
            time.sleep(1)
            #s.send(bytes(g+"\n","utf-8"))
            controlServerSocket.send(drone.getDroneDataSerialized())
		
    except Exception as e:
        print(str(e))
	
    controlServerSocket.close()
    drone.close()
    
    os.kill(videoStreamerProc.pid, signal.SIGKILL)
    videoServerSocket.close()
    
    print('Drone Over')