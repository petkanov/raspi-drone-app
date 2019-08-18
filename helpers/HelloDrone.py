from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
#connString = args.connect
connString = '127.0.0.1:14550'
vehicle = connect(connString, baud=57600, wait_ready=True)

#-- Define function for takeoff
def arm_and_takeoff(tgt_altitude):
    print("Arming motors")
    
    while not vehicle.is_armable:
        print("Waiting for armable status")
        time.sleep(1)
        
    vehicle.mode = VehicleMode("GUIDED")        
    vehicle.armed = True
    
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)
    
    print("Takeoff")
    vehicle.simple_takeoff(tgt_altitude)
    
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        altitude = vehicle.location.global_relative_frame.alt
        
        if altitude >= tgt_altitude - 1:
            print("Altitude reached")
            break
        
        time.sleep(1)
        
#--------  MAIN PROGRAM  ----------

print("Putti===================================")
arm_and_takeoff(30)  
      
#-- Set default speed
vehicle.airspeed = 15

#-- Go to WayPoint1 wp1
wp1 = LocationGlobalRelative(35.9835656, -95.8754125, 10)
vehicle.simple_goto(wp1)


wp2 = LocationGlobalRelative(36.9839361, -95.8721414, 10)
vehicle.simple_goto(wp2)

wp3 = LocationGlobalRelative(35.9835656, -95.8754125, 10)
vehicle.simple_goto(wp3)

print("Putting to sleep program for 15sec")
time.sleep(15)

print("Coming back")
vehicle.mode = VehicleMode("LAND")

print("Putting to sleep program for 15sec")
time.sleep(15)
vehicle.close()

