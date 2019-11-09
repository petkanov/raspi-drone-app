from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import time, threading, logging
import ProtoData_pb2 as proto
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18,GPIO.OUT)
GPIO.output(18,GPIO.HIGH)


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
       self.drone.send_mavlink(msg)
       self.drone.flush()
      
   def executeChangesNow(self):
       msg = self.drone.message_factory.set_position_target_local_ned_encode(
       0,       # time_boot_ms (not used)
       0, 0,    # target system, target component
       mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
       0b0000111111000111, # type_mask (only positions enabled)
       0, 0, 0,
       self.controlTab.speedX, 0, self.controlTab.speedZ, # x, y, z velocity in m/s
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
                  self.controlTab.togleLights()
              
              if self.controlTab.speedX != 0 or self.controlTab.speedZ != 0:
                  msg = self.drone.message_factory.set_position_target_local_ned_encode(
                  0,       # time_boot_ms (not used)
                  0, 0,    # target system, target component
                  mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
                  0b0000111111000111, # type_mask (only positions enabled)
                  0, 0, 0,
                  self.controlTab.speedX, 0, self.controlTab.speedZ, # x, y, z velocity in m/s
                  0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                  0, 0)
        
                  self.drone.send_mavlink(msg)
                  self.drone.flush()
              time.sleep(1.5)
              
          except Exception as e:
              print("Engine killed: "+str(e))
              break
 

class ControlTab:
    def __init__(self, droneControl):
        self.drone = droneControl.drone
        self.droneControl = droneControl
        self.lightState = GPIO.HIGH
        self.startAltitude = 2
        self.speedX = 0
        self.speedZ = 0
        self.incrementValueX = 0.3
        self.incrementValueZ = 0.1
        self.rotationAngle = 15
        self.engine = Engine(droneControl, self)
        self.engine.start()
        
    def stopMovement(self):
        self.speedX = 0
        self.speedZ = 0
        self.engine.executeChangesNow()
        
    def rotateLeft(self):
        self.engine.rotate(-1, self.rotationAngle)
    
    def rotateRight(self):
        self.engine.rotate(1, self.rotationAngle)
        
    def increaseSpeedX(self):
        self.speedX = self.speedX + self.incrementValueX
        self.engine.executeChangesNow()
        
    def decreaseSpeedX(self):
        self.speedX = self.speedX - self.incrementValueX
        self.engine.executeChangesNow()
        
    def stopSpeedX(self):
        self.speedX = 0
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
        
    def armAndTakeoff(self):
        print("Arming")
        
        self.drone.mode = VehicleMode("GUIDED")        
        self.drone.armed = True
    
        while not self.drone.armed:
            print('self.drone.armed: '+str(self.drone.armed))
            self.drone.armed = True
            time.sleep(1)
            
        print("Takeoff")
        self.drone.simple_takeoff(self.startAltitude)
    
        while True:
            currentHight = self.drone.location.global_relative_frame.alt
        
            if currentHight >= self.startAltitude * 0.95:
                print("Altitude reached")
                #commanding movement to the same location to unlock Yaw
                self.drone.simple_goto(self.drone.location.global_relative_frame)
                break
            time.sleep(1)

    def togleLights(self):
        if self.lightState == GPIO.LOW:
            self.lightState = GPIO.HIGH
            GPIO.output(18,GPIO.HIGH)
        else:
            self.lightState = GPIO.LOW
            GPIO.output(18,GPIO.LOW)
        
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
        print("Landing")
        self.drone.mode = VehicleMode("LAND")

class Drone:
    def __init__(self, ip, port, video_port, drone_id):
        #self.drone = connect(ip+":"+str(port), baud=57600, wait_ready=True)
        self.drone = connect('/dev/ttyS0', wait_ready=True, baud=57600)
        self.video_port = video_port
        self.drone_id = drone_id
        self.state = "DISARMED"
        self.controlTab = ControlTab(self)
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
        
    def freeze(self):
        print('Freezing in a spot')
        self.controlTab.stopMovement()
        
    def goHome(self):
        self.controlTab.land()
    
    def togleLights(self):
        self.controlTab.togleLights()

    def executeCommand(self, command):
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
            self.controlTab.rotateLeft()
        if command.code == 3:
            self.controlTab.rotateRight()
        if command.code == 10:
            self.controlTab.land()
            self.state = "LAND"
        if command.code == 11:
            self.controlTab.increaseSpeedX()
        if command.code == 4:
            self.controlTab.decreaseSpeedX()
        if command.code == 12:
            self.controlTab.stopSpeedX()
        if command.code == 13:
            self.controlTab.stopSpeedZ()
        if command.code == 14:
            self.state = "ON MISSION"
            self.controlTab.activateMission(command.data)
        if command.code == 6:
            self.state = "MISSION CANCEL"
            self.controlTab.cancelMission()
            self.freeze()
            
    def close(self):
       self.drone.close()