import socket, os, signal, psutil
from subprocess import Popen
import threading
import time
import ProtoData_pb2 as proto
from drone import Drone 
from utils import Utils


class DataReceiver (threading.Thread):
   def __init__(self, socket, drone):
      threading.Thread.__init__(self)
      self.daemon = True
      self.socket = socket
      self.drone = drone
      
   def run(self):
      while(True):
          try:
              data = Utils.readNetworkMessage(self.socket)
              
              command = proto.Command()
              command.ParseFromString(data)

              self.drone.executeCommand(command)
              
          except Exception as e:
              print("Receiver killed: "+str(e))
              break



#CONTROL_HOST = '192.168.170.48'
CONTROL_HOST = '91.230.195.104'
VIDEO_SERVER_PORT = 1313
DRONE_CLOUD_SERVER_PORT = 1314
DRONE_NAME = 'Peter-Drone-1'

if __name__ == '__main__':
    
    videoStreamerProc = Popen('python3 video_streamer.py --port='+str(VIDEO_SERVER_PORT)+
                                                       ' --name='+str(DRONE_NAME), shell=True)

    controlServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while True:
        try:
            controlServerSocket.connect((CONTROL_HOST, DRONE_CLOUD_SERVER_PORT))
            break;
        except Exception as e:
            print('Could not connect to remote Server: '+str(e))
            time.sleep(2)

    drone = Drone("192.168.170.57", 14550, 11111)
    
    serverMessageReceiver = DataReceiver(controlServerSocket, drone)
    serverMessageReceiver.start()


    while(True):
        try:
            msg = Utils.createNetworkMessage(drone.getDroneDataSerialized())
            controlServerSocket.send(msg)
            
            if videoStreamerProc.poll() == 0: 
               time.sleep(0.5)
               videoStreamerProc = Popen('python3 video_streamer.py --port='+str(VIDEO_SERVER_PORT)+
                                                                  ' --name='+str(DRONE_NAME), shell=True)
               continue

            time.sleep(1)
		
        except Exception as e:
            print(str(e))
            break
        
    os.kill(videoStreamerProc.pid, signal.SIGKILL)
    controlServerSocket.close()
    drone.close()
    
    print('Drone Over')