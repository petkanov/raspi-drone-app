import socket
import threading
import time
import queue 
import ProtoData_pb2 as proto
from drone import Drone 

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
	

if __name__ == '__main__':
    
    socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while True:
        try:
            socket.connect(("192.168.170.48", 1314))
            break;
        except Exception as e:
            print('Could not connect to remote Server: '+str(e))
            time.sleep(2)

    queue = queue.Queue(maxsize=1024)
    drone = Drone("127.0.0.1", 14550)
    serverMessageReceiver = DataReceiver(socket, queue, drone)
    serverMessageReceiver.start()


    serverMessageConsumer = Consumer(queue, drone)
    serverMessageConsumer.start()
    
    
    try:
        while(True):
            time.sleep(1)
            #s.send(bytes(g+"\n","utf-8"))
            socket.send(drone.getDroneDataSerialized())
		
    except Exception as e:
        print(str(e))
	
    socket.close()
    drone.close()

    print('Done')