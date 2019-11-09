import socket, os, signal, logging
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
      self.netStatus = True
      
   def run(self):
      while(True):
          try:
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
          except Exception as e:
              logging.error(str(e), exc_info=True)
              time.sleep(2)
   
   def isInternetOn(self):
    try:
        urlopen(str('http://87.121.112.111'), timeout=1)
        self.netStatus = True
        return True
    except: 
        logging.error('Network is unreachable')
        self.netStatus = False
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
              logging.error(str(e), exc_info=True)
   
   def stop(self):
       self.isActive = False


CONTROL_HOST = '87.121.112.111'
VIDEO_SERVER_PORT = 1313
DRONE_CLOUD_SERVER_PORT = 1314
DRONE_ID = str(netifaces.ifaddresses('eth0')[netifaces.AF_LINK][0]['addr']).replace(':','')
MAX_RECONNECTION_ATTEMPTS = 180

logging.basicConfig(filename='/home/pi/raspi-drone-app/logs/app.log',
                    filemode='w',
                    level=logging.INFO,
                    format='%(asctime)s -%(levelname)s - %(message)s', datefmt='%d-%b-%y %H:%M:%S')

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
        logging.info('Sending to Drone msg..')
        controlServerSocket.send(msg)
        time.sleep(1)


        

if __name__ == '__main__':
    time.sleep(5)
    logging.debug('App started')
    
    while(True):
        try:
            drone = Drone("192.168.0.102", 14553, 11111, DRONE_ID)
            break
        except Exception as e:
            logging.error(str(e), exc_info=True)
            time.sleep(2)
    
    videoStreamerProc = Popen('/usr/bin/python3 /home/pi/raspi-drone-app/video_streamer.py --port='+str(VIDEO_SERVER_PORT)+
                                                       ' --drone_id='+str(DRONE_ID), shell=True)
    logging.info('Video Stream started')
    
    watchdog = ConnectionWatchdog(drone)
    watchdog.start()
    
    

    controlServerSocket = None 
    serverMessageReceiver = None 
    
    
    while True:
        try:
            controlServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
            connectToControlServer(controlServerSocket)
            logging.info('Socket Connection Opened')
    
            serverMessageReceiver = DataReceiver(controlServerSocket, drone)
            serverMessageReceiver.start()
            
            #startSendingDataToServer(controlServerSocket)
            while(watchdog.netStatus):
                msg = Utils.createNetworkMessage(drone.getDroneDataSerialized())
                logging.info('Sending to Drone msg..')
                controlServerSocket.send(msg)
                time.sleep(1)
            raise Exception('Socket Link Broken')
            
        except Exception as e:
            logging.error(str(e), exc_info=True)
            drone.freeze()
            
            if controlServerSocket != None:
                controlServerSocket.close()
            if serverMessageReceiver != None:
                serverMessageReceiver.stop()
                
            time.sleep(2)
            

    os.kill(videoStreamerProc.pid, signal.SIGKILL)
    controlServerSocket.close()
    drone.close()
    
    logging.info('Drone Closed')