import cv2
import numpy as np
import serial
from picamera.array import PiRGBArray
from picamera import PiCamera
from time import sleep

# Inicializar a comunicação serial
ser = serial.Serial('/dev/tty1', 115200, timeout=1)

# Inicializar a câmera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))

# Definir limites do filtro HSV
hmin, smin, vmin = 55, 0, 160
hmax, smax, vmax = 179, 73, 255

# Variáveis auxiliares
area = 0
auxiliar = []
auxiliar_centroide = []
auxiliar_area = []

# Capturar vídeo
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # Obter a matriz numpy representando o frame
    frame = frame.array

    # Transformar imagem para escala HSV
    imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Parâmetros para a máscara
    lower = np.array([hmin, smin, vmin])
    upper = np.array([hmax, smax, vmax])

    # Aplicar máscara
    mascara = cv2.inRange(imgHSV, lower, upper)

    # Localizar contornos na imagem
    contornos_video, hierarchy_video = cv2.findContours(mascara, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Zerar desenho
    desenho = np.zeros_like(frame)

    # Calcular momentos e encontrar o maior contorno
    for i, cnt in enumerate(contornos_video):
        area = cv2.contourArea(cnt)
        if area >= aux_area:
            aux_area = area
            maior_contorno = i  

    if aux_area > 0:
        M = cv2.moments(contornos_video[maior_contorno])
        cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else 0
        cy = int(M["m01"] / M["m00"]) if M["m00"] != 0 else 0

    # Desenhar contorno e centróide na imagem
    color = (0, 0, 255)
    cv2.drawContours(desenho, contornos_video, maior_contorno, color, 1, 8, hierarchy_video, 0)
    cv2.circle(desenho, (cx, cy), 4, color, -1, 8, 0) 

    # Preencher vetores
    auxiliar_area.append(aux_area)
    auxiliar_centroide.append(cx)
    auxiliar_centroide.append(cy)
    auxiliar.append(auxiliar_centroide)
    auxiliar_centroide = []

    # Enviar dados via UART
    data_to_send = f"{cx},{cy},{aux_area}\n"
    ser.write(data_to_send.encode())

    # Limpar buffer
    ser.flush()

    # Limpar buffer do frame para o próximo
    rawCapture.truncate(0)

# Fechar a comunicação serial
ser.close()