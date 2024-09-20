from pymavlink import mavutil
import time

# Connect to the vehicle using serial port
# Replace '/dev/ttyAMA0' with your serial device and set the baud rate correctly
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

# Wait for the heartbeat to find the system ID
master.wait_heartbeat()

# Arm the vehicle
def arm_drone():
    master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Arming...")

    # Wait until the vehicle is armed
    master.motors_armed_wait()
    print("Drone armed!")

# Takeoff to a certain altitude
def takeoff(altitude):
    master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, altitude
    )
    print(f"Taking off to {altitude} meters...")

    # Wait a bit for the drone to take off
    time.sleep(10)

# Move the drone to a relative position
def move_relative(x, y, z):
    master.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms (not used)
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # NED frame (North-East-Down)
        int(0b110111111000),  # mask to specify position control
        x, y, -z,  # x, y, z position (in meters, relative to home)
        0, 0, 0,  # x, y, z velocity (not used)
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0  # yaw, yaw_rate (not used)
    )
    print(f"Moving to x:{x}, y:{y}, z:{z}")
    time.sleep(10)

# Land the drone
def land():
    master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Landing...")
    time.sleep(10)

def main():
    # Arm the drone
    arm_drone()

    # Takeoff to 10 meters altitude
    takeoff(10)

    # Move 5 meters north (positive x direction in NED)
    move_relative(5, 0, 10)

    # Move 5 meters east (positive y direction in NED)
    move_relative(5, 5, 10)

    # Land the drone
    land()

if __name__ == "__main__":
    main()
