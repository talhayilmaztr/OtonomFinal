import cv2
import numpy as np
from scipy.spatial import distance
import time
from pymavlink import mavutil
from picamera2 import Picamera2

# Görüntü işleme sınıfı (PiCamera 2 kullanarak)
class PiCameraImageReceiver:
    def __init__(self):
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_still_configuration())
        self.picam2.start()
        self.current_frame = None

    def capture_image(self):
        self.current_frame = self.picam2.capture_array()

    def get_current_frame(self):
        return self.current_frame  # Bu fonksiyon görüntüyü ana iş parçacığına döndürmek için

def apply_morphology(mask):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    return mask

def colorProcess(frame):
    red_centers = []
    green_triangles = []
    blue_centers = []

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    red_lower2 = np.array([160, 100, 100])
    red_upper2 = np.array([179, 255, 255])

    lower_green = np.array([30, 40, 40])
    upper_green = np.array([90, 255, 255])

    lower_blue = np.array([90, 100, 100])
    upper_blue = np.array([130, 255, 255])

    colors = {"kirmizi": [(lower_red, upper_red), (red_lower2, red_upper2)], "yesil": [(lower_green, upper_green)],
              "mavi": [(lower_blue, upper_blue)]}

    for color_name, bounds in colors.items():
        mask = None
        for (lower_color, upper_color) in bounds:
            if mask is None:
                mask = cv2.inRange(hsv, lower_color, upper_color)
            else:
                mask |= cv2.inRange(hsv, lower_color, upper_color)

        mask = apply_morphology(mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1840:
                hull = cv2.convexHull(cnt)
                hull_area = cv2.contourArea(hull)
                solidity = float(area) / hull_area if hull_area > 0 else 0
                perimeter = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.08 * perimeter, True)

                if len(approx) == 4 and solidity > 0.8 and color_name != "yesil":
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    center = (x + w / 2, y + h / 2)
                    if color_name == "kirmizi":
                        red_centers.append((x, y, w, h))
                    elif color_name == "mavi":
                        blue_centers.append((x, y, w, h))
                elif len(approx) == 4 and color_name == "yesil":
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    green_triangles.append((x, y, w, h))

    return red_centers, green_triangles, blue_centers, frame

def group_rectangles_by_proximity(rectangles, distance_threshold):
    groups = []
    used = set()

    for i in range(len(rectangles)):
        if i in used:
            continue
        group = [rectangles[i]]
        used.add(i)
        for j in range(i + 1, len(rectangles)):
            if j in used:
                continue
            for (x1, y1, w1, h1) in group:
                x2, y2, w2, h2 = rectangles[j]
                center1 = (x1 + w1 / 2, y1 + h1 / 2)
                center2 = (x2 + w2 / 2, y2 + h2 / 2)
                if distance.euclidean(center1, center2) < distance_threshold:
                    group.append(rectangles[j])
                    used.add(j)
                    break
        groups.append(group)

    return groups

# İleriye doğru hareket etme
def move_forward(master, distance_m):
    try:
        print(f"Moving forward {distance_m} meters...")
        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            int(0b110111111000),
            distance_m, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        print(f"Moved forward {distance_m} meters")
    except Exception as e:
        print(f"Failed to move forward: {e}")
        raise

# Mod değiştirme
def set_mode(master, mode):
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    ack = False
    while not ack:
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()
        if ack_msg['command'] == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            ack = True
    print(f"Uçuş modu {mode} olarak değiştirildi")

# Parametre ayarlama işlemi
def set_param(master, param_id, param_value):
    try:
        print(f"Setting {param_id} to {param_value}")
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            param_id.encode('utf-8'),
            param_value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        time.sleep(1)  # Gecikme eklemek

        while True:
            param = master.recv_match(type='PARAM_VALUE', blocking=True)
            if param.param_id.strip() == param_id:
                if param.param_value == param_value:
                    print(f"{param_id} set to {param_value} successfully")
                else:
                    print(f"Failed to set {param_id} to {param_value}")
                    raise Exception(f"Failed to set {param_id}")
                break
    except Exception as e:
        print(f"Failed to set parameter {param_id}: {e}")
        raise

# Belirli bir yöne hareket etme ve sapmaları düzeltme
def move_towards_target(master, vx, vy, vz):
    try:
        print(f"Moving in direction: vx={vx}, vy={vy}, vz={vz}")
        master.mav.set_position_target_local_ned_send(
            0,  # Timestamp (not used)
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            int(0b110111000111),  # Control velocities
            0, 0, 0,  # Positions (not used)
            vx, vy, vz,  # Velocities in m/s
            0, 0, 0,  # Accelerations (not used)
            0, 0  # Yaw and yaw rate (not used)
        )
    except Exception as e:
        print(f"Failed to move towards target: {e}")
        raise

# İniş işlemini gerçekleştiriyoruz
def land(master):
    try:
        print("Landing...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0,
            0
        )
        print("Descending and landing")
    except Exception as e:
        print(f"Failed to land: {e}")
        raise

def ready_arm_mode(master):
    try:
        print("Arm etme kontrollerinin devre dışı bırakılması sağlanıyor...")
        set_param(master, 'ARMING_CHECK', 0)
        set_param(master, 'DISARM_DELAY', 127)
    except Exception as e:
        print(f"Arm etme için hazırlıkta hata oluştu: {e}")
        raise

# Arming işlemini gerçekleştirme
def go_arm_mode(master):
    try:
        print("Araç arm ediliyor...")
        master.arducopter_arm()
        master.motors_armed_wait()
        print("Araç arm edildi")
    except Exception as e:
        print(f"Aracı arm etmede hata oluştu: {e}")
        raise

# Kalkış işlemi
def takeoff(master, altitude):
    try:
        print(f"{altitude} metreye kalkış yapılıyor...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            altitude
        )
        time.sleep(10)  # Kalkış ve stabilize için bekleme süresi
    except Exception as e:
        print(f"Kalkışta hata oluştu: {e}")
        raise

# Drone hareket algoritması
def drone_movement(master, image_receiver):
    # Şehir tespit algoritması
    while True:
        image_receiver.capture_image()
        frame = image_receiver.get_current_frame()  # PiCamera'dan alınan görüntü
        if frame is None:
            continue  # Görüntü alınmadıysa döngüye devam et

        frame = frame[:, 10:-10]  # Sağ ve sol kenarlardan 10 piksel kes

        red_centers, green_triangles, blue_centers, processed_frame = colorProcess(frame)

        all_rectangles = red_centers + green_triangles + blue_centers
        distance_threshold = 400

        groups = group_rectangles_by_proximity(all_rectangles, distance_threshold)

        for group in groups:
            if len(group) > 1:
                x_min = min(x for x, y, w, h in group)
                y_min = min(y for x, y, w, h in group)
                x_max = max(x + w for x, y, w, h in group)
                y_max = max(y + h for x, y, w, h in group)
                city_name = None

                red_count = sum(1 for (x, y, w, h) in group if (x, y, w, h) in red_centers)
                green_count = sum(1 for (x, y, w, h) in group if (x, y, w, h) in green_triangles)
                blue_count = sum(1 for (x, y, w, h) in group if (x, y, w, h) in blue_centers)

                if red_count == 3 and green_count == 1 and blue_count == 1:
                    city_name = "C Sehri"
                elif red_count == 2 and green_count == 2 and blue_count == 1:
                    city_name = "A Sehri"
                    # A şehri algılandıysa 10 metre ileri git
                    print("A şehri algılandı")
                    time.sleep(5)
                    move_forward(master, 10)
                   
                    
                elif red_count == 1 and green_count == 3 and blue_count == 1:
                    city_name = "B Sehri"
                    # B şehri algılandıysa 10 metre ileri git
                    print("B şehri algılandı")
                    time.sleep(5)
                    move_forward(master, 10)
                    

                if city_name == "C Sehri":
                    # C şehri algılandıysa iniş yap
                    print("C Sehri Algilandi, Inis Yapiliyor.")
                    land(master)
                    break  # Döngüden çık ve iniş yap

        cv2.imshow('Processed Frame', processed_frame)

        if cv2.waitKey(16) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

# Ana fonksiyon
def main():
    image_receiver = PiCameraImageReceiver()  # PiCamera görüntü alıcıyı başlat

    # MAVLink bağlantısını başlat
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()

    ready_arm_mode(master)
    set_mode(master, 'GUIDED')
    go_arm_mode(master)
    takeoff(master, 5)  # 5 metreye kalkış
    move_forward(master, 5)  # 5 metre ileri hareket

    drone_movement(master, image_receiver)

if __name__ == "__main__":
    main()
