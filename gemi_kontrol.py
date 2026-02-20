import RPi.GPIO as GPIO
import time
import sys
import select
import termios
import tty
import os
import cv2
from datetime import datetime
from picamera2 import Picamera2

# ==============================
# PIN TANIMLARI
# ==============================

ENA = 18
IN1 = 23
IN2 = 24

ENB = 19
IN3 = 27
IN4 = 22

# ==============================
# GPIO AYARLARI
# ==============================

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

pwm_sol = GPIO.PWM(ENA, 1000)
pwm_sag = GPIO.PWM(ENB, 1000)

pwm_sol.start(0)
pwm_sag.start(0)

# ==============================
# GLOBAL DEĞİŞKENLER
# ==============================

manuel_mod = False
kayit_aktif = False
aktif_komut = 3  # default: dur
frame_sayac = 0
dataset_klasor = None
komut_dosyasi = None
vidfile = None
cfg_cam_res = (640, 480)
cfg_cam_fps = 30
# ==============================
# KAMERA AYARI (480p 30 FPS)
# ==============================

kamera = Picamera2()
microsec_per_frame = int(1000000 / cfg_cam_fps)
video_config = kamera.create_video_configuration(
    main={"size": cfg_cam_res},
    controls={"FrameDurationLimits": (microsec_per_frame, microsec_per_frame)}
)
kamera.configure(video_config)

# ==============================
# MOTOR FONKSİYONLARI
# ==============================

def ileri(hiz=70):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_sol.ChangeDutyCycle(hiz)
    pwm_sag.ChangeDutyCycle(hiz)

def geri(hiz=70):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_sol.ChangeDutyCycle(hiz)
    pwm_sag.ChangeDutyCycle(hiz)

def sola_don():
    pwm_sol.ChangeDutyCycle(40)
    pwm_sag.ChangeDutyCycle(80)

def saga_don():
    pwm_sol.ChangeDutyCycle(80)
    pwm_sag.ChangeDutyCycle(40)

def dur():
    pwm_sol.ChangeDutyCycle(0)
    pwm_sag.ChangeDutyCycle(0)

# ==============================
# DATASET KAYIT
# ==============================

def kayit_baslat():
    global kayit_aktif, komut_dosyasi, dataset_klasor, frame_sayac, vidfile

    zaman = datetime.now().strftime("%Y%m%d_%H%M%S")
    dataset_klasor = f"dataset_{zaman}"
    os.makedirs(dataset_klasor, exist_ok=True)

    # VideoWriter için codec seçimi (Raspberry Pi'de genelde MJPG daha sorunsuz)
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    vidfile = cv2.VideoWriter(f"{dataset_klasor}/video.avi", fourcc,
                                            cfg_cam_fps, cfg_cam_res)

    if not vidfile.isOpened():
        print("UYARI: Video dosyası açılamadı! (codec / yol hatası olabilir)")

    komut_dosyasi = open(f"{dataset_klasor}/labels.txt", "w+")
    komut_dosyasi.write("ts_millisec,frame,angle\n")

    kamera.start()
    kayit_aktif = True
    frame_sayac = 0

    print("DATASET KAYDI BASLADI")


def kayit_durdur():
    global kayit_aktif, komut_dosyasi, vidfile

    kayit_aktif = False
    komut_dosyasi.close()
    kamera.stop()
    vidfile.release()

    print("DATASET KAYDI DURDU")

# ==============================
# KOMUT GÜNCELLEME
# ==============================

def komut_guncelle(deger):
    global aktif_komut
    aktif_komut = deger

# ==============================
# KLAVYE OKUMA
# ==============================

def klavye_oku():
    dr, dw, de = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

# ==============================
# ANA PROGRAM
# ==============================

if __name__ == "__main__":

    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    print("m: Manuel mod")
    print("o: Otomatik mod")
    print("k: Kayit baslat")
    print("l: Kayit durdur")
    print("w/a/s/d/x: Kontrol")

    try:
        while True:

            tus = klavye_oku()

            if tus == "m":
                manuel_mod = True
                print("MANUEL MOD")

            if tus == "o":
                manuel_mod = False
                print("OTOMATIK MOD")

            if tus == "k" and manuel_mod and not kayit_aktif:
                kayit_baslat()

            if tus == "l" and kayit_aktif:
                kayit_durdur()

            if manuel_mod:

                if tus == "w":
                    ileri()
                    komut_guncelle(0)

                elif tus == "s":
                    geri()
                    komut_guncelle(2)

                elif tus == "a":
                    sola_don()
                    komut_guncelle(-1)

                elif tus == "d":
                    saga_don()
                    komut_guncelle(1)

                elif tus == "x":
                    dur()
                    komut_guncelle(3)

            # ==========================
            # FRAME + LABEL KAYDI
            # ==========================

            if kayit_aktif:
                frame = kamera.capture_array()
                if frame is None:
                    time.sleep(0.001)
                    continue

                # Picamera2 burada RGBA (4 kanal) döndürüyor: (480, 640, 4)
                if len(frame.shape) == 3:
                    if frame.shape[2] == 4:
                        # RGBA -> BGR
                        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                    elif frame.shape[2] == 3:
                        # RGB -> BGR
                        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    else:
                        frame_bgr = frame
                else:
                    frame_bgr = frame

                # Boyutlar uyuşmuyorsa yeniden boyutlandır
                if (frame_bgr.shape[1], frame_bgr.shape[0]) != cfg_cam_res:
                    print("Boyutlar uyuşmuyor, yeniden boyutlandırılıyor...")
                    frame_bgr = cv2.resize(frame_bgr, cfg_cam_res)

                if vidfile is not None and vidfile.isOpened():
                    vidfile.write(frame_bgr)
                else:
                    # Bir kez uyarı vermek genelde yeterli ama basitçe her seferinde yazıyoruz
                    print("UYARI: VideoWriter açık değil, frame yazılamıyor.")

                ts = time.time()
                str = "{},{},{}\n".format(int(ts*1000), frame_sayac, aktif_komut)
                komut_dosyasi.write(str)
                komut_dosyasi.flush()

                frame_sayac += 1

            time.sleep(0.033)  # 30 FPS civarı

    except KeyboardInterrupt:
        print("Program Sonlandirildi")

    finally:
        dur()
        if kayit_aktif:
            kayit_durdur()
        pwm_sol.stop()
        pwm_sag.stop()
        GPIO.cleanup()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
