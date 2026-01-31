import serial
import time
import cv2
import threading
from queue import Queue

# -------------------- Serial --------------------
try:
    arduino = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.1)
    time.sleep(2)
    print("Arduino baglantisi basarili.")
except Exception as e:
    print(f"Arduino baglanamadi: {e}")
    arduino = None

# -------------------- Camera --------------------
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# -------------------- Haar Cascade --------------------
frontal_face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
if frontal_face_cascade.empty():
    print("HATA: haarcascade_frontalface_default.xml yuklenemedi! Dosya yolunu kontrol et.")


# -------------------- PID --------------------
class PIDController:
    """
    Hata (error) üzerinden çalışan PID.
    Anti-windup + dt tabanı + output clamp içerir.
    """
    def __init__(self, kp, ki, kd, integral_limit=50.0, output_limit=15.0, dt_min=0.01, dt_max=0.2):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)

        self.integral_limit = float(integral_limit)
        self.output_limit = float(output_limit)

        self.dt_min = float(dt_min)
        self.dt_max = float(dt_max)

        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_time = None

    def reset(self):
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_time = None

    def update(self, error_x, error_y, current_time):
        # İlk çalıştırma
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error_x = float(error_x)
            self.prev_error_y = float(error_y)
            return 0.0, 0.0

        dt = current_time - self.prev_time
        if dt < self.dt_min:
            dt = self.dt_min
        elif dt > self.dt_max:
            dt = self.dt_max

        error_x = float(error_x)
        error_y = float(error_y)

        # Integral (anti-windup clamp)
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt
        self.integral_x = max(-self.integral_limit, min(self.integral_x, self.integral_limit))
        self.integral_y = max(-self.integral_limit, min(self.integral_y, self.integral_limit))

        # Derivative
        deriv_x = (error_x - self.prev_error_x) / dt
        deriv_y = (error_y - self.prev_error_y) / dt

        # PID output
        out_x = (self.kp * error_x) + (self.ki * self.integral_x) + (self.kd * deriv_x)
        out_y = (self.kp * error_y) + (self.ki * self.integral_y) + (self.kd * deriv_y)

        # Output clamp (motor/servo güvenliği)
        out_x = max(-self.output_limit, min(out_x, self.output_limit))
        out_y = max(-self.output_limit, min(out_y, self.output_limit))

        # Update state
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_time = current_time

        return out_x, out_y


# -------------------- Target Finder --------------------
class FindTarget:
    def __init__(self, xfov: float, yfov: float, frame_resolution: tuple):
        self.xfov = float(xfov)
        self.yfov = float(yfov)
        self.width = int(frame_resolution[0])
        self.height = int(frame_resolution[1])

        # hedef: ekranın ortası
        self.center_x = self.width // 2
        self.center_y = self.height // 2

        # derece/piksel
        self.deg_per_pixel_x = self.xfov / self.width
        self.deg_per_pixel_y = self.yfov / self.height

        self.motor_queue = Queue()
        self.last_send_time = 0.0

        # --- PID ayarları (başlangıç için daha stabil) ---
        # Önce P+D ile stabilite; sonra istersen ki'yi az artır.
        self.pid = PIDController(
            kp=1.2,
            ki=0.02,         # çok küçük
            kd=0.10,
            integral_limit=40.0,
            output_limit=12.0,   # bir güncellemede max 12 derece adım
            dt_min=0.01,
            dt_max=0.2
        )

        # Thread başlat
        threading.Thread(target=self.motor_controller, daemon=True).start()

    def calculate_angle_error(self, x, y):
        """
        Yüz merkezinin ekrandaki merkezden açısal hatası (derece).
        Pozitif/negatif yön tersse pixel_error işaretlerini değiştir.
        """
        pixel_error_x = self.center_x - x
        pixel_error_y = self.center_y - y

        angle_x = pixel_error_x * self.deg_per_pixel_x
        angle_y = pixel_error_y * self.deg_per_pixel_y
        return angle_x, angle_y

    def motor_controller(self):
        """
        Kuyruktan aldığı (delta) komutları Arduino'ya gönderir.
        NOT: Arduino tarafı 'delta' bekliyor varsayımıyla yazıldı.
        """
        while True:
            try:
                if arduino is not None and not self.motor_queue.empty():
                    cmd_x, cmd_y = self.motor_queue.get(timeout=0.1)

                    # Arduino float okuyamıyorsa int(round) kullan
                    data = f"{int(round(cmd_x))},{int(round(cmd_y))}\n"
                    arduino.write(data.encode())

                    # opsiyonel: Arduino ACK okumak
                    if arduino.in_waiting:
                        _ = arduino.readline()

                    self.motor_queue.task_done()
                else:
                    time.sleep(0.01)
            except Exception:
                # seri port hatalarında sessiz geç
                time.sleep(0.02)

    def main(self):
        print("Sistem baslatiliyor... Çıkış için 'ESC' tuşuna basın.")

        while True:
            ret, frame = cam.read()
            if not ret:
                print("Kamera okunamadı!")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            frontal_faces = frontal_face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(60, 60)
            )

            # en büyük yüzü seç
            target_face = None
            max_area = 0

            for (x, y, w, h) in frontal_faces:
                area = w * h
                if area > max_area:
                    max_area = area
                    target_face = (x, y, w, h)

                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

            current_time = time.time()

            if target_face:
                x, y, w, h = target_face
                face_center_x = x + w // 2
                face_center_y = y + h // 2

                # hedef noktayı göster (yeşil)
                cv2.circle(frame, (self.center_x, self.center_y), 5, (0, 255, 0), -1)
                # yüz merkezini göster (kırmızı)
                cv2.circle(frame, (face_center_x, face_center_y), 5, (0, 0, 255), -1)

                # 20 Hz motor güncellemesi
                if current_time - self.last_send_time > 0.05:
                    err_x, err_y = self.calculate_angle_error(face_center_x, face_center_y)

                    # deadband: küçük hatada motoru yorma
                    if abs(err_x) > 2.0 or abs(err_y) > 2.0:
                        cmd_x, cmd_y = self.pid.update(err_x, err_y, current_time)

                        # Kuyruk şişmesin
                        if self.motor_queue.qsize() < 2:
                            self.motor_queue.put((cmd_x, cmd_y))
                            self.last_send_time = current_time

            else:
                # Yüz kaybolunca PID state reset (zıplamayı azaltır)
                self.pid.reset()

            cv2.imshow("Kaan Teknoloji Takip Sistemi", frame)

            if cv2.waitKey(10) == 27:  # ESC
                break

        # temizlik
        cam.release()
        cv2.destroyAllWindows()
        if arduino and arduino.is_open:
            arduino.close()


if __name__ == '__main__':
    # FOV ve çözünürlük
    fT = FindTarget(51.44, 39.11, (640, 480))
    fT.main()
