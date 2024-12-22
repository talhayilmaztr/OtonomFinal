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

# Function to wait for command acknowledgment
def wait_for_ack(command):
    while True:
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()

        if ack_msg['command'] == command:
            if ack_msg['result'] == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"Command {command} acknowledged!")
                return True
            else:
                print(f"Command {command} failed with result: {ack_msg['result']}")
                return False

# Get current position (LOCAL_POSITION_NED)
def get_current_position():
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    pos = msg.to_dict()
    print(f"Current position: x={pos['x']}, y={pos['y']}, z={pos['z']}")
    return pos

# Check system status (battery, sensors)
def get_system_status():
    msg = master.recv_match(type='SYS_STATUS', blocking=True)
    status = msg.to_dict()
    print(f"Battery: {status['battery_remaining']}%, Sensors: {status['onboard_control_sensors_health']}")
    return status

# Monitor heartbeat
def wait_for_heartbeat():
    master.wait_heartbeat()
    print("Heartbeat received from the vehicle")

# Send a command with retry mechanism
def send_command_with_retry(command_func, max_retries=3):
    for attempt in range(max_retries):
        command_func()
        if wait_for_ack(mavutil.mavlink.MAV_CMD_DO_REPOSITION):  # Check the relevant command
            print("Command successful!")
            return True
        else:
            print(f"Attempt {attempt + 1}/{max_retries} failed. Retrying...")
            time.sleep(2)  # Short delay before retrying
    print("Max retries reached. Aborting.")
    return False

# Main logic
def main():
    # Arm the drone
    arm_drone()

    # Takeoff to 10 meters altitude
    takeoff(10)

    # Move 5 meters north (positive x direction in NED)
    send_command_with_retry(lambda: move_relative(5, 0, 10))

    # Move 5 meters east (positive y direction in NED)
    send_command_with_retry(lambda: move_relative(5, 5, 10))

    # Land the drone
    land()

if __name__ == "__main__":
    main()
