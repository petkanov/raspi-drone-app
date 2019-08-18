import threading
import queue
import time
import random

class Consumer (threading.Thread):
   def __init__(self, q):
      threading.Thread.__init__(self)
      self.q = q
   def run(self):
      while(True):
          print(threading.currentThread().getName()+" Before getting..")
          data = self.q.get()
          print("After getting ",data)
          self.q.task_done()
          sec = random.randint(1,10)
          print(threading.currentThread().getName()+" is sleeping secs: "+str(sec))
          time.sleep(sec)
          
		
      
qu = queue.Queue(maxsize=1024)

c = Consumer(qu)
c2 = Consumer(qu)
c.start()
c2.start()

for i in range(0, 12):
    qu.put(random.randint(13,123))
    time.sleep(1)
    
print("DONE PRODUCER")
qu.join()
print("EXIT PROGRAMM")