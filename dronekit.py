from dronekit import connect , VehicleMode
import time
from pymavlink import mavutil
import argparse

def connectMyCopter():
	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()
	
	connection_string = args.connect
	baud_rate = 57600
	vehicle = connect(connection_string,baud=baud_rate,wait_ready=True)
	return vehicle



def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def goto_position_target_local_ned(north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)


def RTL():
    vehicle.mode = VehicleMode("RTL")
    print(" Mode: %s" % vehicle.mode.name)    # settable

    while vehicle.mode.name is not "RTL":
        time.sleep(1)
        print ("Vehicle mode is: %s" % str(vehicle.mode.name))
        vehicle.mode = VehicleMode("RTL")

    print ("Vehicle Mode is : RTL")

vehicle = connectMyCopter()
time.sleep(1)
arm_and_takeoff(2)
goto_position_target_local_ned(2,1,1)
#time.sleep(1)
#goto_position_target_local_ned(0,-1,0)
#time.sleep(1)
#goto_position_target_local_ned(0,0,1)
time.sleep(5)
RTL()
vehicle.close()
