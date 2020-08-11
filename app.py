import socket, os, signal, logging
from subprocess import Popen
import threading, time, netifaces, psutil
import ProtoData_pb2 as proto
from urllib.request import urlopen
from drone import Drone 
from utils import Utils


CONTROL_HOST = '109.121.252.89' # home ip
#CONTROL_HOST = '87.121.112.55'    # VPS ip 
USE_SIMULATOR = True

VIDEO_SERVER_PORT = 1313
DRONE_CLOUD_SERVER_PORT = 1314
DRONE_ID = str(netifaces.ifaddresses('eth0')[netifaces.AF_LINK][0]['addr']).replace(':','')
MAX_RECONNECTION_ATTEMPTS = 180

logging.basicConfig(filename='/home/pi/raspi-drone-app/logs/app'+str(time.asctime())+'.log',
                    filemode='w',
                    level=logging.INFO,
                    format='%(asctime)s -%(levelname)s - %(message)s', datefmt='%d-%b-%y %H:%M:%S')



class ConnectionWatchdog (threading.Thread):
   def __init__(self, drone):
      threading.Thread.__init__(self)
      self.daemon = True
      self.drone = drone
      self.connectionTryCounter = 0;
      self.netStatus = True
      
   def run(self):
      time.sleep(15)
      
      while(True):
          try:
              if self.isInternetOn():
                  self.connectionTryCounter = 0;
                  time.sleep(1)
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
        #urlopen(str('http://216.58.212.46'), timeout=5)
        urlopen(str('http://'+CONTROL_HOST), timeout=5)
        self.netStatus = True
        return True
    except: 
        logging.error('Network is unreachable: ', exc_info=True)
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
              logging.error('app.py line 82: '+str(e), exc_info=True)
   
   def stop(self):
       self.isActive = False



def connectToControlServer(controlServerSocket):
    while True:
        controlServerSocket.connect((CONTROL_HOST, DRONE_CLOUD_SERVER_PORT))
            
        droneIdBytes = Utils.createNetworkMessage(str.encode(DRONE_ID))
        controlServerSocket.send(droneIdBytes)
        logging.info('Connected To Control Server')
        break




if __name__ == '__main__':
    logging.debug('App started')
    
    while(True):
        try:
            drone = Drone("192.168.0.102", 14553, 11111, DRONE_ID, USE_SIMULATOR) # simulator case, 192.168.0.104 is RPi local address
            break
        except Exception as e:
            logging.error(str(e), exc_info=True)
            time.sleep(2)
    
    
    watchdog = ConnectionWatchdog(drone)
    watchdog.start()
    

    controlServerSocket = None 
    serverMessageReceiver = None 
    videoStreamerProc = None
    audioStreamerProc = None
    
    while drone.isActive:
        try:
            while not watchdog.netStatus:
                time.sleep(1)
            
            time.sleep(3)
            
            controlServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
            connectToControlServer(controlServerSocket)
            logging.info('Socket Connection Opened')
    
            videoStreamerProc = Popen('/usr/bin/python3 /home/pi/raspi-drone-app/video_streamer.py --port='+str(VIDEO_SERVER_PORT)+
                                                       ' --ip='+CONTROL_HOST + ' --drone_id='+str(DRONE_ID), shell=True)
            logging.info('Video Stream started')

            audioStreamerProc = Popen('/usr/bin/omxplayer /home/pi/Music/slide.mp3', shell=True)
            logging.info('Audio Stream started')
           
            serverMessageReceiver = DataReceiver(controlServerSocket, drone)
            serverMessageReceiver.start()
            
            while watchdog.netStatus and drone.isActive:
                msg = Utils.createNetworkMessage(drone.getDroneDataSerialized())
                controlServerSocket.send(msg)
                time.sleep(1)
            
            logging.warning('Exiting drone.isActive cycle on line 161')
            #raise Exception('Socket Link Broken')
        except Exception as e:
            logging.error(str(e), exc_info=True)
            drone.freeze()
            
        finally:
            if videoStreamerProc != None:
               current_process = psutil.Process(videoStreamerProc.pid)
               children = current_process.children(recursive=True)
               for child in children:
                   if child.pid != os.getpid():
                       os.kill(child.pid, signal.SIGKILL)
               os.kill(videoStreamerProc.pid, signal.SIGKILL)
            
            if controlServerSocket != None:
                controlServerSocket.close()
            if serverMessageReceiver != None:
                serverMessageReceiver.stop()
            

    controlServerSocket.close()
    drone.close()
    
    logging.info('Drone Closed')