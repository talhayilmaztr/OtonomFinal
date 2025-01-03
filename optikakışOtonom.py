import cv2
import numpy as np
from picamera2 import Picamera2
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

# PiCamera2 yapılandırması
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
picam2.configure(camera_config)
picam2.start()

# Pixhawk ile bağlantı kur
vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)  # Pixhawk bağlantı noktası ve baud hızı

# Arming ve kalkış fonksiyonu
def arm_and_takeoff(aTargetAltitude):
    print("Arming motors")
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable...")
        time.sleep(1)
    
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f} meters")
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Renk aralıklarını belirleme (HSV formatında)
red_range = (np.array([0, 120, 70]), np.array([10, 255, 255]))
green_range = (np.array([36, 50, 70]), np.array([89, 255, 255]))
blue_range = (np.array([90, 50, 70]), np.array([128, 255, 255]))

# Renk filtresi fonksiyonu
def get_color_mask(image, color_range):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # Görüntüyü HSV formatına çevir
    lower_bound, upper_bound = color_range  # Alt ve üst renk sınırlarını al
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)  # Belirtilen renk aralığında maske uygula
    return mask

# Drone'u belirli hızlarda hareket ettiren fonksiyon
def send_ned_velocity(vehicle, vx, vy, vz, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # Kontrol maskesi (yalnızca hız vektörü kullan)
        0, 0, 0,  # Pozisyonu kullanmıyoruz
        vx, vy, vz,  # İleri/geri, sağ/sol, yukarı/aşağı hız
        0, 0, 0,  # Hızlanma (kullanılmıyor)
        0, 0
    )
    for _ in range(duration):
        vehicle.send_mavlink(msg)
        vehicle.flush()

# Şekil tespiti fonksiyonu
def detect_shapes(image, mask, color_name):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centers = []
    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
        if len(approx) == 3:
            shape = "triangle"
        elif len(approx) == 4:
            shape = "square"
        else:
            continue

        # Şekil merkezini hesapla
        M = cv2.moments(contour)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            centers.append((cx, cy))
            cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)
            cv2.putText(image, f"{color_name} {shape}", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return centers

# Optik Akış Hesaplama Fonksiyonu
def calculate_optical_flow(prev_frame, current_frame):
    # Görüntüleri gri tonlamaya çevir
    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    current_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

    # Farneback Optical Flow ile hareket yönlerini hesapla
    flow = cv2.calcOpticalFlowFarneback(prev_gray, current_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)

    # X ve Y yönlerindeki akışı ayır
    flow_x, flow_y = flow[..., 0], flow[..., 1]

    # Ortalama akış miktarını hesapla
    avg_flow_x = np.mean(flow_x)
    avg_flow_y = np.mean(flow_y)

    return avg_flow_x, avg_flow_y

# Ana fonksiyon
def main():
    arm_and_takeoff(10)  # Drone 10 metreye kadar yükselsin

    prev_frame = None  # İlk kare için boş değişken

    while True:
        # Kameradan görüntü al
        frame = picam2.capture_array()

        # İlk kareyi sakla (ilk kare optik akış için referans olacak)
        if prev_frame is None:
            prev_frame = frame
            continue

        # Optik akış hesapla
        avg_flow_x, avg_flow_y = calculate_optical_flow(prev_frame, frame)

        # Optik akışa göre hareket yönlerini belirle
        print(f"Optical Flow - X: {avg_flow_x}, Y: {avg_flow_y}")

        # Mavi kutu tespiti
        blue_mask = get_color_mask(frame, blue_range)
        centers = detect_shapes(frame, blue_mask, "blue")

        # Eğer merkez tespit edildiyse, drone hareket ettir
        if len(centers) > 0:
            # Görüntü çözünürlüğü
            image_center_x = frame.shape[1] // 2
            image_center_y = frame.shape[0] // 2

            # Algılanan nesnenin merkezini al
            cx, cy = centers[0]

            # X ve Y eksenlerindeki sapmaları hesapla
            x_offset = cx - image_center_x
            y_offset = cy - image_center_y

            # Eğer nesne merkezde değilse drone'u hareket ettir
            vx = vy = 0

            # X ekseni sapması için drone'u sağa/sola hareket ettir
            if abs(x_offset) > 30:  # Merkezde küçük bir tolerans
                vy = -0.5 if x_offset > 0 else 0.5

            # Y ekseni sapması için drone'u ileri/geri hareket ettir
            if abs(y_offset) > 30:
                vx = -0.5 if y_offset > 0 else 0.5

            # Drone'u belirtilen hızlarda hareket ettir
            send_ned_velocity(vehicle, vx, vy, 0, 1)

        # Sonuçları ekranda göster
        cv2.imshow("Tracking Blue Shape", frame)

        # 'q' tuşuna basarak çıkış yapabilirsiniz
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Şu anki kareyi bir sonraki kare için referans yap
        prev_frame = frame

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
