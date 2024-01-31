#!/usr/bin/env python3
# подключаем библиотеки
import rospy
from std_msgs.msg import Bool
import numpy as np
import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
import time

def detect_marker_position(frame, marker_size, camera_matrix, dist_coeffs):
    """
    Определяет положение камеры относительно маркера.

    Параметры:
    - frame: кадр из видеопотока.
    - marker_size: размер маркера в метрах.
    - camera_matrix: матрица камеры.
    - dist_coeffs: коэффициенты дисторсии камеры.
    """

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ###################################
    # Ecли не работает этот код:
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)    
    parameters = aruco.DetectorParameters()
    ####################################
    # Расскрыть комменты в этом, а выше закомментировать
    # aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    # parameters = aruco.DetectorParameters_create()
    ####################################
    
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
        inv_tvec = -tvec
        return rvec, inv_tvec, corners, ids
    else:
        return None, None, corners, ids

def main():
    # Создаем новый узел с именем 'talker' и указываем, что он является анонимным
    rospy.init_node('extra_land', anonymous=True)
    # Создаем новый паблишер на тему 'land' с типом сообщения 'std_msgs/String'
    publisher = rospy.Publisher('extra_land', Bool, queue_size=10)
    # Устанавливаем частоту публикации сообщений в 10 Гц
    rate = rospy.Rate(5)
    # Создание фигуры для отображения графика и текущего кадра    
    fig, ax1 = plt.subplots(figsize=(10, 5))
    # Настройка осей
    ax1.set_xlabel('Pixels W')
    ax1.set_ylabel('Pixels H')
    ax1.set_title('Detected Marker')
    # Значения матрицы камеры строго индивидуальны для каждой камеры
    #camera_matrix = np.array([[640, 0, 320], [0, 640, 240], [0, 0, 1]], dtype=np.float32)
    camera_matrix = np.array([[1063.08787,0,728.51113], [0,1082.59904,351.74316],[0,0,1]],dtype=np.float32)
    #dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
    dist_coeffs = np.array([[0.497793, -0.508361, 0.061278, 0.054812, 0]], dtype=np.float32)
    # ИНДЕКС ВАШЕЙ КАМЕРЫ МОЖЕТ БЫТЬ ДРУГОЙ, не '0' :
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        ret, frame = cap.read()
        frame = cv2.resize(frame, (640, 640))
        im = ax1.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    else:
        exit(1)
    flag_exit = False
    while cap.isOpened():
        ret, frame = cap.read()
        frame = cv2.resize(frame, (640, 640))
        if not ret:
            break

        rvecs, tvecs, corners, ids = detect_marker_position(frame, 0.03, camera_matrix, dist_coeffs)
        if rvecs is not None:
            frame = aruco.drawDetectedMarkers(frame, corners, ids)  # рисуем обнаруженные маркеры
            for idx, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
                frame = cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)  # рисуем оси на маркере
            flag_exit = True
        rate.sleep()
        # Обновляем изображение на графике
        im.set_data(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        plt.pause(0.01) # Пауза для обновления графика
        if flag_exit:
           # сохраняем для контроля кадр идентификации aruco маркера
           # cv2.imwrite('aruco_detected.jpg', frame)
           # посылаем сигнал о экстренной посадке 10 раз и выходим
           for i in range(10):              
              publisher.publish(False)
              rate.sleep()
           # задержка для окончания посадки 
           # time.sleep(20)
           break
        # продолжаем мониторинг
        publisher.publish(True)
        rate.sleep()
    # Закрываем видеофайл и освобождаем ресурсы
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main() # Запускаем основную функцию
