import socket, os, signal
from subprocess import Popen
import threading, time, netifaces
import ProtoData_pb2 as proto
from urllib.request import urlopen
from drone import Drone 
from utils import Utils


class ConnectionWatchdog (threading.Thread):
   def __init__(self, drone):
      threading.Thread.__init__(self)
      self.daemon = True
      self.drone = drone
      self.connectionTryCounter = 0;
      
   def run(self):
      while(True):
          if self.isInternetOn():
              self.connectionTryCounter = 0;
              time.sleep(2)
          else:
              self.drone.freeze()
              self.connectionTryCounter = self.connectionTryCounter + 1
              time.sleep(1)
              if self.connectionTryCounter == MAX_RECONNECTION_ATTEMPTS:
                  drone.goHome()
                  break
   
   def isInternetOn(self):
    try:
        urlopen(str('http://google.com'), timeout=1)
        return True
    except: 
        return False

class CommandData:
   def __init__(self):
      self.code = None
      self.data = None      
       
class DataReceiver (threading.Thread):
   def __init__(self, socket, drone):
      threading.Thread.__init__(self)
      self.daemon = True
      self.socket = socket
      self.drone = drone
      self.isActive = True
      
   def run(self):
      while(self.isActive):
          try:
              data = Utils.readNetworkMessage(self.socket)
              
              command = proto.Command()
              command.ParseFromString(data)
              
              commandData = CommandData()
              commandData.code = command.code
              
              if(command.code == 14):
                  missionData = proto.MissionData()
                  missionData.ParseFromString(command.payload)
                  
                  commandData.data = missionData
              
              self.drone.executeCommand(commandData)
              
          except Exception as e:
              print("Receiver killed: "+str(e))
              break
      print("----------- DataReceiver done -----------------")
   
   def stop(self):
       self.isActive = False


CONTROL_HOST = '87.121.112.111'
VIDEO_SERVER_PORT = 1313
DRONE_CLOUD_SERVER_PORT = 1314
DRONE_ID = str(netifaces.ifaddresses('eth0')[netifaces.AF_LINK][0]['addr']).replace(':','')
MAX_RECONNECTION_ATTEMPTS = 180


def connectToControlServer(controlServerSocket):
    print('connectToControlServer')
    while True:
        controlServerSocket.connect((CONTROL_HOST, DRONE_CLOUD_SERVER_PORT))
            
        droneIdBytes = Utils.createNetworkMessage(str.encode(DRONE_ID))
        controlServerSocket.send(droneIdBytes)
        break

def startSendingDataToServer(controlServerSocket):
    print('startSendingDataToServer')
    while(True):
        msg = Utils.createNetworkMessage(drone.getDroneDataSerialized())
        controlServerSocket.send(msg)
        time.sleep(1)


        

if __name__ == '__main__':
    time.sleep(5)
    
    drone = Drone("192.168.0.102", 14553, 11111, DRONE_ID)
    
    videoStreamerProc = Popen('/usr/bin/python3 /home/pi/raspi-drone-app/video_streamer.py --port='+str(VIDEO_SERVER_PORT)+
                                                       ' --drone_id='+str(DRONE_ID), shell=True)
    
    watchdog = ConnectionWatchdog(drone)
    watchdog.start()

    controlServerSocket = None 
    serverMessageReceiver = None 
    
    
    while True:
        try:
            controlServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            connectToControlServer(controlServerSocket)
    
            serverMessageReceiver = DataReceiver(controlServerSocket, drone)
            serverMessageReceiver.start()
            
            startSendingDataToServer(controlServerSocket)
            
        except Exception as e:
            print('error: '+str(e))
            drone.freeze()
            
            if controlServerSocket != None:
                controlServerSocket.close()
            if serverMessageReceiver != None:
                serverMessageReceiver.stop()
                
            time.sleep(2)
            

    os.kill(videoStreamerProc.pid, signal.SIGKILL)
    controlServerSocket.close()
    drone.close()
    
    print('Drone Over')