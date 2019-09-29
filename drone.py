from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time, threading
import ProtoData_pb2 as proto


class XEngine (threading.Thread):
   def __init__(self, drone):
      threading.Thread.__init__(self)
      self.daemon = True
      self.drone = drone
      self.command_strength = 0
   
   def increaseStrength(self):
      self.command_strength += 1;
      
   def decreaseStrength(self):
      self.command_strength -= 1;
      
   def stopMovement(self):
       self.command_strength = 0;
      
   def run(self):
      while(True):
          try:
              if self.command_strength != 0:
                 self.drone.XMove(self.command_strength)
              time.sleep(1)
              
          except Exception as e:
              print("XEngine killed: "+str(e))
              break

class ZEngine (threading.Thread):
   def __init__(self, drone):
      threading.Thread.__init__(self)
      self.daemon = True
      self.drone = drone
      self.command_strength = 0
   
   def increaseStrength(self):
      self.command_strength += 1;
      
   def decreaseStrength(self):
      self.command_strength -= 1;
      
   def stopMovement(self):
       self.command_strength = 0;
      
   def run(self):
      while(True):
          try:
              if self.command_strength != 0:
                 self.drone.ZMove(self.command_strength)
              time.sleep(1)
              
          except Exception as e:
              print("ZEngine killed: "+str(e))
              break


class Drone:
    def __init__(self, ip, port, video_port, drone_id):
        self.drone = connect(ip+":"+str(port), baud=57600, wait_ready=True)
        self.video_port = video_port
        self.drone_id = drone_id
        self.airspeed = 50
        self.rotationAngle = 5
        self.state = "DISARMED"
        self.x_engine = XEngine(self)
        self.z_engine = ZEngine(self)
        self.x_engine.start()
        self.z_engine.start()
        print("Drone connected")
        
    def getDroneDataSerialized(self):
        droneData = proto.DroneData()
        
        droneData.altitude = self.drone.location.global_relative_frame.alt
        droneData.latitude = self.drone.location.global_relative_frame.lat
        droneData.longitude = self.drone.location.global_relative_frame.lon
        droneData.voltage = self.drone.battery.voltage
        droneData.speed = float(self.drone.airspeed)
        droneData.state = self.state
        droneData.video_port = int(self.video_port)
        droneData.drone_id = str(self.drone_id)
        
        return droneData.SerializeToString() 
        
    def armAndReady(self):
        print("Arming Drone.. ")

        self.state = "ARMING"
        while not self.drone.is_armable:
            time.sleep(1)
        
        self.drone.mode = VehicleMode("GUIDED")        
        self.drone.armed = True
    
        while not self.drone.armed:
            time.sleep(1)
            
        self.state = "TAKING OFF"
        print("Drone Armed and Ready to Takeoff")
        self.takeoff(10)
        
    def takeoff(self, hight):
        print("Takeoff")
        self.drone.simple_takeoff(hight)
    
        while True:
            currentHight = self.drone.location.global_relative_frame.alt
        
            if currentHight >= hight * 0.95:
                print("Altitude reached")
                #commanding movement to the same location to unlock Yaw
                self.drone.simple_goto(self.drone.location.global_relative_frame)
                break
            time.sleep(1)
        self.state = "READY"
            
    def ZMove(self, velocity_strength):
        msg = self.drone.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            0, 0, velocity_strength,     #-- VELOCITY  vx, vy, vz,
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
        
        self.drone.send_mavlink(msg)
        self.drone.flush()
        
    def rotateLeft(self):
        msg = self.drone.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            self.rotationAngle,  # param 1, yaw in degrees
            5,          # param 2, yaw speed deg/s
            -1,          # param 3, direction -1 ccw, 1 cw
            True, # param 4, 1 - relative to current position offset, 0 - absolute, angle 0 means North
            0, 0, 0)    # param 5 ~ 7 not used
        self.drone.send_mavlink(msg)
        self.drone.flush()
        
    def rotateRight(self):
        msg = self.drone.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            self.rotationAngle,    # param 1, yaw in degrees
            5,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            True, # param 4, 1 - relative to current position offset, 0 - absolute, angle 0 means North
            0, 0, 0)    # param 5 ~ 7 not used
        self.drone.send_mavlink(msg)
        self.drone.flush()
        
    def XMove(self, velocity_strength):
        msg = self.drone.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only positions enabled)
        0, 0, 0,
        velocity_strength, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)
        
        self.drone.send_mavlink(msg)
        self.drone.flush()
        
    def land(self):
        print("Landing")
        self.drone.mode = VehicleMode("LAND")
        self.state = "LAND"
    
    def executeCommand(self, command):
        if command.code == 9:
            self.armAndReady()
        if command.code == 1:
            self.z_engine.increaseStrength()
        if command.code == 5:
            self.z_engine.decreaseStrength()
        if command.code == 2:
            self.rotateLeft()
        if command.code == 3:
            self.rotateRight()
        if command.code == 10:
            self.land()
        if command.code == 11:
            self.x_engine.increaseStrength()
        if command.code == 4:
            self.x_engine.decreaseStrength()
        if command.code == 12:
            self.x_engine.stopMovement()
        if command.code == 13:
            self.z_engine.stopMovement()
            
    def close(self):
       self.drone.close()