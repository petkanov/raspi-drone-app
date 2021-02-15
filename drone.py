from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import time, threading, logging, math
import ProtoData_pb2 as proto
import RPi.GPIO as GPIO

from servocontroller import ServoController

dropperPIN = 21
igniterPIN = 2

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(dropperPIN,GPIO.OUT)
GPIO.output(dropperPIN,GPIO.HIGH)

GPIO.setup(igniterPIN,GPIO.OUT)
GPIO.output(igniterPIN,GPIO.HIGH)

class Engine (threading.Thread):
   def __init__(self, droneControl, controlTab):
      threading.Thread.__init__(self)
      self.daemon = True
      self.droneControl = droneControl
      self.drone = droneControl.drone
      self.lastMissionCmndIndex = -1
      self.controlTab = controlTab
      
      logging.info('Engine started')
      
   def rotate(self, direction, rotationAngle):
       msg = self.drone.message_factory.command_long_encode(
       0, 0,    # target system, target component
       mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
       0, #confirmation
       rotationAngle,  # param 1, yaw in degrees
       1,          # param 2, yaw speed deg/s
       direction,          # param 3, direction -1 ccw, 1 cw
       True, # param 4, 1 - relative to current position offset, 0 - absolute, angle 0 means North
       0, 0, 0)    # param 5 ~ 7 not used
       """
       msg = self.drone.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        0, # Target system
        0, # Target component
        0b00000001,
        self.to_quaternion(0.0, 0.0, 0.0), # Quaternion
        0.0, # Body roll rate in radian
        0.0, # Body pitch rate in radian
        2, # Body yaw rate in radian/second
        0.5  # Thrust
        )
       """
       self.drone.send_mavlink(msg)
       self.drone.flush()
       
       
   def to_quaternion(self, roll=0.0, pitch=0.0, yaw=0.0):
     """Convert degrees to quaternions."""
     t0 = math.cos(yaw * 0.5)
     t1 = math.sin(yaw * 0.5)
     t2 = math.cos(roll * 0.5)
     t3 = math.sin(roll * 0.5)
     t4 = math.cos(pitch * 0.5)
     t5 = math.sin(pitch * 0.5)
     w = t0 * t2 * t4 + t1 * t3 * t5
     x = t0 * t3 * t4 - t1 * t2 * t5
     y = t0 * t2 * t5 + t1 * t3 * t4
     z = t1 * t2 * t4 - t0 * t3 * t5
     return [w, x, y, z]    
       
       
      
   def killMotorsNow(self):
       msg = self.drone.message_factory.command_long_encode(
       1, 1,    # target system, target component
       mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION , #command
       1, #confirmation
       1,  # param 1, yaw in degrees
       1,          # param 2, yaw speed deg/s
       1,          # param 3, direction -1 ccw, 1 cw
       True, # param 4, 1 - relative to current position offset, 0 - absolute, angle 0 means North
       0, 0, 0)    # param 5 ~ 7 not used
       self.drone.send_mavlink(msg)
       self.drone.flush()
       
   def executeChangesNow(self):
       msg = self.drone.message_factory.set_position_target_local_ned_encode(
       0,       # time_boot_ms (not used)
       0, 0,    # target system, target component
       mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
       0b0000111111000111, # type_mask (only positions enabled)
       0, 0, 0,
       self.controlTab.speedX, self.controlTab.speedY, self.controlTab.speedZ, # x, y, z velocity in m/s
       0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
       0, 0)
        
       self.drone.send_mavlink(msg)
       self.drone.flush()
   
   def run(self):
      while(True):
          try:
              if self.drone.commands.next == self.lastMissionCmndIndex:
                  self.droneControl.state = 'MISSION OVER'
                  self.drone.commands.next = 0
                  self.lastMissionCmndIndex = -1
                  self.drone.mode = VehicleMode("GUIDED")
                  #self.controlTab.togleLights()
              
              if self.controlTab.speedX != 0 or self.controlTab.speedY != 0 or self.controlTab.speedZ != 0:
                  msg = self.drone.message_factory.set_position_target_local_ned_encode(
                  0,       # time_boot_ms (not used)
                  0, 0,    # target system, target component
                  mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
                  0b0000111111000111, # type_mask (only positions enabled)
                  0, 0, 0,
                  self.controlTab.speedX, self.controlTab.speedY, self.controlTab.speedZ, # x, y, z velocity in m/s
                  0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                  0, 0)
        
                  self.drone.send_mavlink(msg)
                  self.drone.flush()
              time.sleep(1.5)
              
          except Exception as e:
              logging.error("Engine killed: "+str(e))
              break
 

class ControlTab:
    def __init__(self, droneControl):
        self.drone = droneControl.drone
        self.droneControl = droneControl
        
        self.lightState = GPIO.HIGH
        self.ignitorState = GPIO.HIGH
        
        self.cameraAngle = 140 # 60 - Lowest ; 230 Max value;
        self.servoCamera = ServoController(self.cameraAngle)
        self.servoCamera.start()
        
        self.startAltitude = 1.5
        self.speedX = 0
        self.speedY = 0
        self.speedZ = 0
        self.incrementValueX = 0.5
        self.incrementValueY = 0.5
        self.incrementValueZ = 0.2
        self.rotationAngle = 10
        self.engine = Engine(droneControl, self)
        self.engine.start()
        
    def stopMovement(self):
        self.speedX = 0
        self.speedZ = 0
        self.engine.executeChangesNow()
        
    def rotateLeft(self, angle):
        self.engine.rotate(-1, angle)
    
    def rotateRight(self, angle):
        self.engine.rotate(1, angle)
        
    def increaseSpeedX(self):
        self.speedX = self.speedX + self.incrementValueX
        self.engine.executeChangesNow()
        
    def decreaseSpeedX(self):
        self.speedX = self.speedX - self.incrementValueX
        self.engine.executeChangesNow()
   
    def leftSpeedY(self):
        self.speedY = self.speedY - self.incrementValueY
        self.engine.executeChangesNow()
        
    def rightSpeedY(self):
        self.speedY = self.speedY + self.incrementValueY
        self.engine.executeChangesNow()
        
    def stopSpeedXY(self):
        self.speedX = 0
        self.speedY = 0
        self.engine.executeChangesNow()
        
    def increaseSpeedZ(self):
        self.speedZ = self.speedZ - self.incrementValueZ
        self.engine.executeChangesNow()
        
    def decreaseSpeedZ(self):
        self.speedZ = self.speedZ + self.incrementValueZ
        self.engine.executeChangesNow()
        
    def stopSpeedZ(self):
        self.speedZ = 0
        self.engine.executeChangesNow()
        
    def killMotorsNow(self):
        self.engine.killMotorsNow()
        
    def armAndTakeoff(self):
        logging.info("Arming") 
        
        self.drone.mode = VehicleMode("GUIDED")   

        self.drone.armed = True
        time.sleep(1)
    
        while not self.drone.armed:
            logging.info('self.drone.armed: '+str(self.drone.armed))
            self.drone.armed = True
            time.sleep(1)
        
        self.drone.simple_takeoff(self.startAltitude)
        logging.info("Takeoff")

        while True:
            currentHight = self.drone.location.global_relative_frame.alt
        
            if currentHight >= self.startAltitude * 0.95:
                logging.info("Altitude reached")
                #commanding movement to the same location to unlock Yaw
                self.drone.simple_goto(self.drone.location.global_relative_frame)
                break
            time.sleep(1)

    def togleLights(self):
        
        self.ignitorState = GPIO.LOW
        GPIO.output(igniterPIN,GPIO.LOW)
        time.sleep(1)
        self.ignitorState = GPIO.HIGH
        GPIO.output(igniterPIN,GPIO.HIGH)
        
        self.lightState = GPIO.LOW
        GPIO.output(dropperPIN,GPIO.LOW)
        time.sleep(1)
        self.lightState = GPIO.HIGH
        GPIO.output(dropperPIN,GPIO.HIGH) 

    def cameraUP(self):
        if self.cameraAngle > 60:
            self.cameraAngle = self.cameraAngle - 20
            print(self.cameraAngle)
            self.servoCamera.setAngle(self.cameraAngle)

    def cameraDOWN(self):
        if self.cameraAngle < 220:
            self.cameraAngle = self.cameraAngle + 20
            print(self.cameraAngle)
            self.servoCamera.setAngle(self.cameraAngle)
        
    def cancelMission(self):
        self.drone.mode = VehicleMode("GUIDED")
        self.drone.commands.next = 0
        self.engine.lastMissionCmndIndex = -1
        cmds = self.drone.commands
        cmds.clear()
        cmds.upload()
        
        
    def activateMission(self, pointsData):
        
        if self.drone.mode == VehicleMode("AUTO"):
            self.drone.mode = VehicleMode("GUIDED")
            self.droneControl.state = "MISSION PAUSE"
            self.droneControl.freeze()
            return
        
        if self.drone.commands.next > 0 and self.drone.mode == VehicleMode("GUIDED"):
            self.droneControl.state = "MISSION RESUME"
            self.drone.mode = VehicleMode("AUTO")
            return
            
        
        self.drone.commands.next = 0
        cmds = self.drone.commands
        cmds.clear()
        # add & remove command because of the bug(not seeing first time added command)
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                          mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                          float(0), float(0), float(0)  ))
        cmds.clear()
        
        if not self.drone.armed:
            self.armAndTakeoff()
        
        for point in pointsData.point:
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                             mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                             float(point.latitude), float(point.longitude), float(point.altitude)
                    ))
        #add dummy waypoint (to let us know we have reached the destination)
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                          mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                          float(0), float(0), float(0)  ))
        self.engine.lastMissionCmndIndex = cmds.count
        cmds.upload()
        
        self.drone.mode = VehicleMode("AUTO")
        
    def land(self):
        logging.info("Landing")
        self.drone.channels.overrides = {}
        self.drone.mode = VehicleMode("LAND")

class Drone:
    def __init__(self, ip, port, video_port, drone_id, use_simulator):
        if use_simulator:
            self.drone = connect(ip+":"+str(port), baud=57600, wait_ready=True)
        else:
            self.drone = connect('/dev/ttyS0', wait_ready=True, baud=57600)
            
        self.video_port = video_port
        self.drone_id = drone_id
        self.state = "DISARMED"
        self.isActive =True
        self.controlTab = ControlTab(self)
        logging.info("Drone connected")
        
    def getDroneDataSerialized(self):
        droneData = proto.DroneData()
        if self.drone.location.global_relative_frame.alt != None:
          droneData.altitude = self.drone.location.global_relative_frame.alt
        if self.drone.location.global_relative_frame.lat != None:		  
          droneData.latitude = self.drone.location.global_relative_frame.lat
        if self.drone.location.global_relative_frame.lon != None:		  
          droneData.longitude = self.drone.location.global_relative_frame.lon
        if self.drone.battery.voltage != None:
          droneData.voltage = self.drone.battery.voltage
        if self.drone.airspeed != None:
          droneData.speed = float(self.drone.airspeed)
        droneData.state = self.state
        droneData.video_port = int(self.video_port)
        droneData.drone_id = str(self.drone_id)
        return droneData.SerializeToString() 
        
    def freeze(self):
        logging.info('Freezing in a spot')
        self.controlTab.stopMovement()
        
    def goHome(self):
        print('Going Home')
        for i in range(0,10):
            self.controlTab.increaseSpeedZ()
            time.sleep(0.5)
        
        while True:
            currentHight = self.drone.location.global_relative_frame.alt
        
            if currentHight >= 70 * 0.95:
                logging.info("Safe RTL Altitude reached")
                self.drone.mode = VehicleMode("RTL")
                break
            time.sleep(1)
    
    def togleLights(self):
        self.controlTab.togleLights()

    def executeCommand(self, command):
        print(command.code)
        
        if command.code == 7:
            self.goHome()
        if command.code == 8:
            self.togleLights()
        if command.code == 9:
            self.state = "ARMING"
            self.controlTab.armAndTakeoff()
            self.state = "READY"
        if command.code == 1:
            self.controlTab.increaseSpeedZ()
        if command.code == 5:
            self.controlTab.decreaseSpeedZ()
        if command.code == 2:
            self.controlTab.rotateLeft(10)
        if command.code == 3:
            self.controlTab.rotateRight(10)
        if command.code == 18:
            self.controlTab.rotateLeft(45)
        if command.code == 19:
            self.controlTab.rotateLeft(90)
        if command.code == 20:
            self.controlTab.rotateRight(45)
        if command.code == 21:
            self.controlTab.rotateRight(90)
        
        
        if command.code == 22:
            self.controlTab.cameraUP()
        if command.code == 23:
            self.controlTab.cameraDOWN()
            
            
            
            
        if command.code == 10:
            self.controlTab.land()
            self.state = "LAND"
        if command.code == 11:
            self.controlTab.increaseSpeedX()
        if command.code == 4:
            self.controlTab.decreaseSpeedX()
        if command.code == 16:
            self.controlTab.rightSpeedY()
        if command.code == 15:
            self.controlTab.leftSpeedY()
        if command.code == 12:
            self.controlTab.stopSpeedXY()
        if command.code == 13:
            self.controlTab.stopSpeedZ()
        if command.code == 14:
            self.state = "ON MISSION"
            self.controlTab.activateMission(command.data)
        if command.code == 6:
            self.state = "MISSION CANCEL"
            self.controlTab.cancelMission()
            self.freeze()
        if command.code == 17:
            self.state = "MOTORS KILL"
            self.controlTab.killMotorsNow()
            self.isActive = False
            
    def close(self):
       self.drone.close()