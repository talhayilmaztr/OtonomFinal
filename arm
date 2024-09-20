import time
from pymavlink import mavutil

# Dron ile bağlantı kurma
def connect_mavlink(connection_string):
    vehicle = mavutil.mavlink_connection(connection_string, baud=57600)
    vehicle.wait_heartbeat()  # Bağlantı başarılı olduğunda kalp atışı sinyali alır
    print("Bağlantı başarılı!")
    return vehicle

# Motorları arm etme
def arm_vehicle(vehicle):
    print("Motorlar arm ediliyor...")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    vehicle.motors_armed_wait()  # Motorlar arm olana kadar bekle
    print("Motorlar arm edildi!")

# İniş (LAND) komutu
def land(vehicle):
    print("İniş yapılıyor...")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )

# Ana program
if __name__ == "__main__":
    connection_string = '/dev/ttyAMA0'  # Bağlantı string'inizi buraya girin
    vehicle = connect_mavlink(connection_string)

    # Motorları arm et
    arm_vehicle(vehicle)

    # 5 saniye bekle
    time.sleep(5)

    # İniş komutu ver
    land(vehicle)

    # Bağlantıyı kapat
    vehicle.close()
