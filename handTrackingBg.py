import threading
import time
import math
import mediapipe as mp
import cv2
import serial
import socket
import mouse
import win32api
import win32.lib.win32con as win32con
import keyboard
import numpy as np

# Настройки:
handsOnlyMode = True # Дали да показва само ръцете или прегледа от камерата
numberOfHands = 2 # Брой ръце за засичане
cs = False # Неизползвано, вероятно за Counter-Strike специфични настройки
controlMouse = False # Дали да се контролира мишката на компютъра или не
tcp = False  # Изпращане на данни към Steam драйвера
devMode = True # Превключва показването, когато пръстите са вдигнати и показва номерата на всяка точка
bt = False # Дали да се опита да се свърже с Bluetooth или не
controlMode = 7 # Превключване с цифрите
disabled = False # Дали скриптът е деактивиран
xRes = 800 # Резолюция по X (възможни: 1280 800 640)
yRes = 450 # Резолюция по Y (възможни: 720 450 360)
resMulti = 1.6 # Множител на резолюцията (1 1.6 2 ако промените резолюцията, трябва да промените и това спрямо резолюцията, за да съответства винаги на 1280)

# Разширени настройки:
base_scale = 100  # Базов коефициент на мащабиране
# depth_multiplier = 6.5  # Регулирайте това за фина настройка на чувствителността
depth_multiplier = 60  # Регулирайте това за фина настройка на чувствителността
tiltNegativeMin = -65 # Минимален отрицателен наклон (затворени стойности -60, нормални стойности -80)
tiltNegativeMax = -20 # Максимален отрицателен наклон (затворени стойности -25, нормални стойности -40)
tiltPositiveMin= 25 # Минимален положителен наклон (затворени стойности 25, нормални стойности 15)
tiltPositiveMax= 60 # Максимален положителен наклон (затворени стойности 60, нормални стойности 40)

if bt:
  # Свързване към Arduino чрез сериен порт (COM12 в примера)
  arduino = serial.Serial(port='COM12', baudrate=9600, timeout=.1) # 7 (вероятно номерът на COM порта)

# Използване на MediaPipe за рисуване на скелета на ръката върху идентифицираните ръце в реално време
drawingModule = mp.solutions.drawing_utils
handsModule = mp.solutions.hands

# Използване на функционалност на CV2 за създаване на видео поток и добавяне на стойности
cap = cv2.VideoCapture(0)
# fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v') # За запис на видео (не се използва)

# Изпращане на данните към C++ драйвера чрез TCP сокет
if(tcp):
  client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  client_socket.connect(("localhost", 12345))

fingers = [0, 0, 0, 0, 0] # Състояние на пръстите (0=свит, 1=изпънат)
# clicked = {"Left": False, "Right": False}  # Проследява състоянието на щипване за всяка ръка (не се използва)
clickedLeft = False # Дали лявата ръка е в състояние "щипване"
clickedRight = False # Дали дясната ръка е в състояние "щипване"
detect_pinchVar = None # Променлива за състоянието на щипването (не се използва активно)
# treashhold = 65 # референтна стойност (не се използва директно)

# Инициализиране на променливи за позиции на ключови точки
pos0, pos1, pos2, pos3, pos4, pos5, pos6, pos8, pos9, pos10, pos12, pos13, pos14, pos16, pos17, pos18, pos20, prevPosL, prevPosR, currentPos = [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0] # Инициализиране на позиции на ключови точки
# Инициализиране на други променливи
handPosX, handPosY, refreshes, clicked, prev_frame_time, new_frame_time, fps, tilt, tiltP, tiltPLast, tiltLast, tiltClosed = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 # refreshes по подразбиране 5
stopped, steer, wait = True, False, False # Състояния за управление
normal_vector = [0, 0, 0] # Нормален вектор на дланта
font = cv2.FONT_HERSHEY_SIMPLEX # Шрифт за текст върху изображението

def get_hand_type(hand_landmarks, width, height):
    # Определя дали ръката е лява или дясна спрямо центъра на изображението
    if hand_landmarks.landmark[mp.solutions.hands.HandLandmark.WRIST].x * width < width / 2:
        return 'Left'
    else:
        return 'Right'

def detect_pinch(hand_landmarks, hand_type, pos4, pos8):
    # Засича "щипване" (pinch) между палеца и показалеца
    global clickedLeft
    global clickedRight
    pos4_lm = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.THUMB_TIP]
    pos8_lm = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP]
    # Изчислява разстоянието между върховете на палеца и показалеца
    distance = math.sqrt((pos4_lm.x - pos8_lm.x) ** 2 + (pos4_lm.y - pos8_lm.y) ** 2)
    # print(distance)
    if(hand_type == "Left"):
      # Проверява за щипване на лявата ръка
      if distance < 0.05 and not clickedLeft:
          clickedLeft = True
          if tcp:
            client_socket.send(str("pl" + "\n").encode())  # Изпрати като низ (pinch left)
          else:
            return "pl" # Връща "pl" (pinch left)
          # print(f"{hand_type} hand pinch detected") # Отпечатва съобщение за щипване
      elif distance >= 0.05 and clickedLeft: # Използва се 0.05, за да се избегне трептене
          clickedLeft = False
          if tcp:
            client_socket.send(str("u" + "\n").encode())  # Изпрати като низ (unpinch)
          return "ul" # (unpinch left)
    elif(hand_type == "Right"):
      # Проверява за щипване на дясната ръка
      if distance < 0.05 and not clickedRight:
          clickedRight = True
          if tcp:
            client_socket.send(str("pr" + "\n").encode())  # Изпрати като низ (pinch right)
          else:
            return "pr" # Връща "pr" (pinch right)
          # print(f"{hand_type} hand pinch detected") # Отпечатва съобщение за щипване
      elif distance >= 0.05 and clickedRight: # Използва се 0.05, за да се избегне трептене
          clickedRight = False
          if tcp:
            client_socket.send(str("u" + "\n").encode())  # Изпрати като низ (unpinch)
          else:
            return "ur" # (unpinch right)

def sendPos(h, x, y, ry, rx, rz):
  # Изпраща позиция и ротация на ръката през TCP сокета
  client_socket.send(str("H" + h + "\n").encode())  # Изпрати като низ (Hand L/R)
  client_socket.send(str("Y" + str(y) + "\n").encode())  # Изпрати като низ (Y позиция)
  client_socket.send(str("X" + str(abs(x - xRes/2) * resMulti) + "\n").encode())  # Изпрати като низ (X позиция, коригирана)
  client_socket.send(str("RX" + str(rx * -1) + "\n").encode())  # Изпрати като низ (X ротация, инвертирана)
  client_socket.send(str("RY" + str(ry * -1) + "\n").encode())  # Изпрати като низ (Y ротация, инвертирана)
  client_socket.send(str("RZ" + str(rz) + "\n").encode())  # Изпрати като низ (Z ротация)

# Добавяне на стойности за увереност и допълнителни настройки към MediaPipe проследяването на ръце
with handsModule.Hands(static_image_mode=False, min_detection_confidence=0.7, min_tracking_confidence=0.5, max_num_hands=numberOfHands) as hands:
    while True:
        ret, frame = cap.read() # Четене на кадър от камерата

        # frame1 = cv2.flip(cv2.resize(frame, (xRes, yRes)), 1) # намалете за по-добра производителност, обърнат огледално
        frame1 = cv2.resize(frame, (xRes, yRes)) # Преоразмеряване на кадъра, не е обърнат огледално
        frame2 = frame1 - frame1 # Създаване на черен кадър със същите размери

        # Обработка на кадъра с MediaPipe Hands
        results = hands.process(cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)) # наслагва информацията за ръцете
        # Изчисляване на FPS (кадри в секунда)
        new_frame_time = time.time()
        fps = str(int(1/(new_frame_time-prev_frame_time)))
        prev_frame_time = new_frame_time

        # Показване на FPS
        if not handsOnlyMode:
            cv2.putText(frame1, fps, (0, 18), font, 0.75, (0, 255, 0), 2, cv2.LINE_AA) # Върху оригиналния кадър
        else:
            cv2.putText(frame2, fps, (0, 18), font, 0.75, (0, 255, 0), 2, cv2.LINE_AA) # Върху черния кадър

        # Проверка дали са открити ръце
        if results.multi_hand_landmarks and results.multi_handedness:
            # Итериране през всяка открита ръка
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                hand_type = handedness.classification[0].label # Вземане на типа ръка (Left/Right)
                # print(f"Hand type: {hand_type}") # извеждане на типа ръка

                # Рисуване на ключовите точки на ръката върху кадъра
                drawingModule.draw_landmarks(frame1, hand_landmarks, handsModule.HAND_CONNECTIONS) # Върху оригиналния кадър
                drawingModule.draw_landmarks(frame2, hand_landmarks, handsModule.HAND_CONNECTIONS) # Върху черния кадър

                # Изчисляване на центъра на ръката (приблизително)
                handPosX = int((hand_landmarks.landmark[0].x * xRes + hand_landmarks.landmark[5].x * xRes + hand_landmarks.landmark[9].x * xRes + hand_landmarks.landmark[13].x * xRes + hand_landmarks.landmark[17].x * xRes) / 5)
                handPosY = int((hand_landmarks.landmark[0].y * yRes + hand_landmarks.landmark[5].y * yRes + hand_landmarks.landmark[9].y * yRes + hand_landmarks.landmark[13].y * yRes + hand_landmarks.landmark[17].y * yRes) / 5)
                # Интерполиране на наклона (tilt) за Z ротация (-1 до 1)
                tiltZ = np.interp(tilt, [-120, 120], [-1, 1])
                # print(tiltZ) # извеждане на tiltZ

                # Рисуване на синя точка за лявата ръка и червена точка за дясната ръка и изпращане на информация
                if hand_type == 'Left':
                  if(tcp):
                    # Изпращане на позиция и ротация за лявата ръка
                    sendPos("L", handPosX, handPosY, normal_vector[0], normal_vector[1], tiltZ)
                  # Рисуване на точка в центъра на ръката
                  if not handsOnlyMode:
                    cv2.circle(frame1, (handPosX, handPosY), 4, (255, 0, 0), -1) # Синя точка
                  else:
                    cv2.circle(frame2, (handPosX, handPosY), 4, (255, 0, 0), -1) # Синя точка
                else: # Ако е дясна ръка
                  if(tcp):
                    # Изпращане на позиция и ротация за дясната ръка
                    sendPos("R", handPosX, handPosY, normal_vector[0], normal_vector[1], tiltZ)
                  # Рисуване на точка в центъра на ръката
                  if not handsOnlyMode:
                    cv2.circle(frame1, (handPosX, handPosY), 4, (0, 0, 255), -1) # Червена точка
                  else:
                    cv2.circle(frame2, (handPosX, handPosY), 4, (0, 0, 255), -1) # Червена точка

                # Итериране през всички ключови точки на ръката
                for point in handsModule.HandLandmark:
                  normalizedLandmark = hand_landmarks.landmark[point] # Нормализирани координати (0-1)
                  # Изчисляване на нормалния вектор на дланта (използвайки китка, основа на показалец, основа на кутре)
                  points = np.array([
                      [hand_landmarks.landmark[0].x, hand_landmarks.landmark[0].y, hand_landmarks.landmark[0].z],
                      [hand_landmarks.landmark[5].x, hand_landmarks.landmark[5].y, hand_landmarks.landmark[5].z],
                      [hand_landmarks.landmark[17].x, hand_landmarks.landmark[17].y, hand_landmarks.landmark[17].z]
                  ])
                  normal_vector = np.cross(points[2] - points[0], points[1] - points[2]) # Векторно произведение
                  normal_vector /= np.linalg.norm(normal_vector) # Нормализиране на вектора
                  # print(normal_vector) # извеждане на нормалния вектор

                  # Преобразуване на нормализираните координати в пикселни координати
                  pixelCoordinatesLandmark = drawingModule._normalized_to_pixel_coordinates(normalizedLandmark.x, normalizedLandmark.y, xRes, yRes)
                  if pixelCoordinatesLandmark is None: # Ако преобразуването е неуспешно, прескочи
                      break
                  # print(point.value, hand_landmarks.landmark[point.value]) # извеждане на стойността на точката и нейните координати

                  # Показване на номера на точката до всяка ключова точка (ако devMode е активен)
                  if pixelCoordinatesLandmark and devMode:
                    cv2.putText(frame1, str(point.value), pixelCoordinatesLandmark, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(frame2, str(point.value), pixelCoordinatesLandmark, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                  # Изчисляване на приблизителна Z дълбочина (не се използва активно)
                  # z_depth = abs((hand_landmarks.landmark[0].z + hand_landmarks.landmark[3].z + hand_landmarks.landmark[6].z + hand_landmarks.landmark[9].z + hand_landmarks.landmark[12].z + hand_landmarks.landmark[15].z + hand_landmarks.landmark[17].z + hand_landmarks.landmark[20].z) / 8) # Изчисляване на средната Z дълбочина
                  # Алтернативен метод за дълбочина чрез разлика в X координати
                  z_depth = abs(pos5[0] - pos0[0]) - abs(pos17[0] - pos0[0]) # точки 5-0 17-0
                  # cv2.circle(frame2, (z_depth, 400), 4, (255, 0, 0), -1) # рисуване на кръг за z_depth
                  # Изчисляване на праг за щипване, базиран на дълбочината (не се използва активно)
                  # treashhold = int(float((-100 * float(hand_landmarks.landmark[8].z)) * 6.5)) # референтна стойност
                  treashhold = int(abs(z_depth * depth_multiplier / base_scale)) # Изчисляване на праг


                  # Актуализиране на позициите за различните точки
                  if point == handsModule.HandLandmark.WRIST: pos0 = pixelCoordinatesLandmark
                  if point == handsModule.HandLandmark.THUMB_CMC: pos1 = pixelCoordinatesLandmark # Не се използва
                  if point == handsModule.HandLandmark.THUMB_MCP: pos2 = pixelCoordinatesLandmark
                  if point == handsModule.HandLandmark.THUMB_IP: pos3 = pixelCoordinatesLandmark
                  if point == handsModule.HandLandmark.THUMB_TIP:
                    pos4 = pixelCoordinatesLandmark
                    # Определяне дали палецът е свит или изпънат (спрямо предходната става)
                    if pos4[0] < pos3[0]: # Проверка по X координата (може да е неточно за лява/дясна ръка)
                        fingers[0] = 1 # Палец изпънат
                    else:
                        fingers[0] = 0 # Палец свит
                  if point == handsModule.HandLandmark.INDEX_FINGER_MCP: pos5 = pixelCoordinatesLandmark
                  # Показалец
                  if point == handsModule.HandLandmark.INDEX_FINGER_PIP: pos6 = pixelCoordinatesLandmark
                  if point == handsModule.HandLandmark.INDEX_FINGER_TIP:
                    pos8 = pixelCoordinatesLandmark
                    # Определяне дали показалецът е свит или изпънат (спрямо предходната става)
                    if pos8[1] > pos6[1]: # Проверка по Y координата (по-ниска Y = по-нагоре)
                      fingers[1] = 1 # Показалец изпънат
                    else:
                      fingers[1] = 0 # Показалец свит
                    # Контролира мишката на компютъра и жестовете за VR
                    # print(detect_pinch(hand_landmarks, hand_type, pos4, pos8)) # Проверка за щипване
                  
                  if(controlMouse):
                    # Call detect_pinch and store the result
                    pinch_result = detect_pinch(hand_landmarks, hand_type, pos4, pos8)
                    # Update detect_pinchVar only if the result is not None
                    if pinch_result is not None:
                        detect_pinchVar = pinch_result

                    # Коментиран код за управление на мишката
                    if(cs): # Преместване на мишката
                      win32api.mouse_event(win32con.MOUSEEVENTF_MOVE | win32con.MOUSEEVENTF_ABSOLUTE, int((0 + (currentPos[0] - prevPos[0]))/win32api.GetSystemMetrics(0)*11111), int((0 + (prevPos[1] - currentPos[1]))/win32api.GetSystemMetrics(1)*11111) ,0 ,0) # настройки за CSGO
                    elif(not cs and detect_pinchVar == "pr" and hand_type == "Right"): # Преместване на мишката (не се използва активно)
                      currentPos = [handPosX, handPosY] # Присвояване на текущата позиция
                      # win32api.mouse_event(win32con.MOUSEEVENTF_MOVE | win32con.MOUSEEVENTF_ABSOLUTE, int((mousePos[0] + (prevPos[0] - handPosX))/win32api.GetSystemMetrics(0)*65555), int((mousePos[1] + (handPosY - prevPos[1]))/win32api.GetSystemMetrics(1)*65555) ,0 ,0) # нормална употреба
                      mouse.move(int(prevPosR[0] - currentPos[0]), int (currentPos[1] - prevPosR[1]), absolute=False, duration=0) # Преместване на мишката (не се използва активно)

                    # Коментиран код за ляв клик
                    elif(detect_pinchVar == "pl" and hand_type == "Left"): # Проверка за щипване
                      currentPos = [handPosX, handPosY] # Присвояване на текущата позиция
                      mouse.press(button='left')
                      mouse.move(int(prevPosL[0] - currentPos[0]), int (currentPos[1] - prevPosL[1]), absolute=False, duration=0) # Преместване на мишката (не се използва активно)
                    elif(detect_pinchVar == "ul"): # Проверка за щипване
                      mouse.release(button='left')

                    # Актуализиране на предишната позиция
                    if(hand_type == "Right"): # Проверка за дясна ръка и контрол на мишката
                      prevPosR = [handPosX, handPosY] # присвояване prevPos = [handPosX, handPosY]
                    elif(hand_type == "Left"): # Проверка за лява ръка и контрол на мишката
                      prevPosL = [handPosX, handPosY]

                  if point == handsModule.HandLandmark.MIDDLE_FINGER_MCP: pos9 = pixelCoordinatesLandmark
                  # Среден пръст
                  if point == handsModule.HandLandmark.MIDDLE_FINGER_PIP: pos10 = pixelCoordinatesLandmark
                  if point == handsModule.HandLandmark.MIDDLE_FINGER_TIP:
                    pos12 = pixelCoordinatesLandmark
                    # Определяне дали средният пръст е свит или изпънат
                    if pos12[1] > pos10[1]:
                      fingers[2] = 1 # Среден пръст изпънат
                    else:
                      fingers[2] = 0 # Среден пръст свит
                  if point == handsModule.HandLandmark.RING_FINGER_MCP: pos13 = pixelCoordinatesLandmark
                  # Безименен пръст
                  if point == handsModule.HandLandmark.RING_FINGER_PIP: pos14 = pixelCoordinatesLandmark
                  if point == handsModule.HandLandmark.RING_FINGER_TIP:
                    pos16 = pixelCoordinatesLandmark
                    # Определяне дали безименният пръст е свит или изпънат
                    if pos16[1] > pos14[1]:
                      fingers[3] = 1 # Безименен пръст изпънат
                    else:
                      fingers[3] = 0 # Безименен пръст свит
                  if point == handsModule.HandLandmark.PINKY_MCP: pos17 = pixelCoordinatesLandmark
                  # Малък пръст (кутре)
                  if point == handsModule.HandLandmark.PINKY_PIP: pos18 = pixelCoordinatesLandmark
                  if point == handsModule.HandLandmark.PINKY_TIP:
                    pos20 = pixelCoordinatesLandmark
                    # Определяне дали кутрето е свит или изпънат
                    if pos20[1] > pos18[1]:
                      fingers[4] = 1 # Кутре изпънато
                    else:
                      fingers[4] = 0 # Кутре свито

                  # Брояч за забавяне (на всеки 50 кадъра)
                  if(refreshes == 50): # Обикновено се използва за контрол на мишката, сега за всякакви необходими закъснения (като изпращане на данни)
                    refreshes = 0 # Нулиране на брояча
                    wait = False # Флаг за изчакване (не се използва активно)
                    # currentPos = prevPos # Коментирано присвояване
                  elif(not refreshes == 50):
                    refreshes = refreshes + 1 # Инкрементиране на брояча
                    # prevPos = [handPosX, handPosY] # Коментирано присвояване


                  # Запазване на предишните стойности на наклона
                  tiltLast = tilt
                  tiltClosedLast = tiltClosed
                  # Изчисляване на наклона (tilt) на ръката
                  # tilt = math.degrees(math.atan2(pos9[0] - pos12[0], pos9[1] - pos12[1])) # Изчисляване на вектора за средния пръст (коментар)
                  # tilt = math.degrees(math.atan2(pos0[0] - pos9[0], pos0[1] - pos9[1])) # Изчисляване на вектора за китката (коментар)
                  # Изчисляване на вектора от китката (pos0) до средната точка между кокалчетата на средния (pos9) и безименния (pos13) пръст
                  tilt = math.degrees(math.atan2(pos0[0] - (pos9[0] + pos13[0]) / 2, pos0[1] - (pos9[1] + pos13[1]) / 2)) # Изчисляване на вектора от китката до кокалчетата за контрол със стиснат юмрук
                  tiltClosed = tilt # Запазване на същата стойност (не се използва различно tiltClosed)

                  # Логика за управление чрез Bluetooth (ако е активирано)
                  # 0 горе, 1 долу (за състоянието на пръстите)
                  if(bt and not disabled): # Проверка дали Bluetooth е активен и скриптът не е деактивиран
                    # Различни режими на управление (избира се с controlMode)
                    if(controlMode == 1): # юмрук напред, един пръст нагоре назад
                      # Управление на движението и спирането
                      if(tilt > -40 and tilt < 20 and not fingers[1] and not fingers[2] and not fingers[3] and not fingers[4] and (stopped == False or (stopped and steer))): # дясна ръка (юмрук)
                        # arduino.write(bytes(str(tilt), 'utf-8')) # изпращане на tilt към Arduino (коментар)
                        stopped = True # Спри
                        steer = False # Не завивай
                        print("S") # Отпечатай "S" (Stop)
                        if bt:
                          arduino.write(bytes("s", 'utf-8')) # Изпрати "s" към Arduino
                      elif(tilt > -40 and tilt < 20 and stopped and fingers[1] and fingers[2] and fingers[3] and fingers[4]): # Отворена ръка
                        stopped = False # Движи се
                        steer = False # Не завивай
                        print("F") # Отпечатай "F" (Forward)
                        if bt:
                          arduino.write(bytes("F", 'utf-8')) # Изпрати "F" към Arduino
                      # Управление на завиването
                      elif(tilt > 20 and not steer): # Наклон надясно
                        stopped = True # Спри (докато завива)
                        steer = True # Завивай
                        print("R") # Отпечатай "R" (Right)
                        if bt:
                          arduino.write(bytes("R", 'utf-8')) # Изпрати "R" към Arduino
                      elif(tilt < -40 and not steer): # Наклон наляво
                        stopped = True # Спри (докато завива)
                        steer = True # Завивай
                        print("L") # Отпечатай "L" (Left)
                        if bt:
                          arduino.write(bytes("L", 'utf-8')) # Изпрати "L" към Arduino
                      # Движение назад (един пръст)
                      elif(not fingers[1] and fingers[2] and fingers[3] and fingers[4] and stopped): # Само показалец свит
                        stopped = False # Движи се
                        print("B") # Отпечатай "B" (Backward)
                        if bt:
                          arduino.write(bytes("B", 'utf-8')) # Изпрати "B" към Arduino

                    elif(controlMode == 2): # обратното на първата опция, но малко по-добре (почти същото сега)
                      # Подобен на режим 1, но с различни условия за F/B
                      if(tilt > -40 and tilt < 20 and not fingers[1] and not fingers[2] and not fingers[3] and not fingers[4] and stopped == False): # дясна ръка (юмрук)
                        # arduino.write(bytes(str(tilt), 'utf-8')) # коментар
                        stopped = True # Спри
                        if bt:
                          arduino.write(bytes("s", 'utf-8')) # Изпрати "s"
                      elif(tilt > 20): # Наклон надясно
                        stopped = False # Може да се движи и завива едновременно?
                        print("R")
                        if bt:
                          arduino.write(bytes("R", 'utf-8')) # Изпрати "R"
                      elif(tilt < -40): # Наклон наляво
                        stopped = False # Може да се движи и завива едновременно?
                        print("L")
                        if bt:
                          arduino.write(bytes("L", 'utf-8')) # Изпрати "L"
                      # Движение напред (показалец свит)
                      elif(not fingers[1] and  fingers[2] and fingers[3] and fingers[4] and stopped):
                        stopped = False
                        print("F")
                        if bt:
                          arduino.write(bytes("F", 'utf-8')) # Изпрати "F"
                      # Движение назад (кутре изпънато)
                      elif(fingers[1] and fingers[2] and fingers[3] and not fingers[4] and stopped):
                        stopped = False
                        print("B")
                        if bt:
                          arduino.write(bytes("B", 'utf-8')) # Изпрати "B"

                    elif(controlMode == 3): # юмрук с палец отпред = назад, юмрук с палец отвън = напред
                      if(tilt > -40 and tilt < 20 and stopped == False and not fingers[1] and not fingers[2] and not fingers[3] and not fingers[4]): # дясна ръка (юмрук)
                        # arduino.write(bytes(str(tilt), 'utf-8')) # коментар
                        stopped = True # Спри
                        steer = False # Не завивай
                        print("S")
                        if bt:
                          arduino.write(bytes("s", 'utf-8')) # Изпрати "s"
                      elif(tilt > -40 and tilt < 20 and fingers[1] and fingers[2] and fingers[3] and fingers[4] and stopped or (tilt > -40 and tilt < 20 and steer)): # Отворена ръка или след завой
                        stopped = False # Движи се
                        steer = False # Не завивай
                        if(not fingers[0]): # Палец свит (отпред)
                          print("F")
                          if bt:
                            arduino.write(bytes("F", 'utf-8')) # Изпрати "F"
                        else: # Палец изпънат (отвън)
                          print("B")
                          if bt:
                            arduino.write(bytes("B", 'utf-8')) # Изпрати "B"
                      elif(tilt > 20 and not steer): # Наклон надясно
                        stopped = False # Продължава движение докато завива?
                        steer = True # Завивай
                        print("R")
                        if bt:
                          arduino.write(bytes("R", 'utf-8')) # Изпрати "R"
                      elif(tilt < -40 and not steer): # Наклон наляво
                        stopped = False # Продължава движение докато завива?
                        steer = True # Завивай
                        print("L")
                        if bt:
                          arduino.write(bytes("L", 'utf-8')) # Изпрати "L"

                    elif(controlMode == 4): # същото като 3, но с аналогов наклон (изпраща стойност за завиване)
                      # Спиране с юмрук
                      if(tilt > tiltNegativeMax and tilt < tiltPositiveMin and not fingers[1] and not fingers[2] and not fingers[3] and not fingers[4] and (stopped == False or (stopped and steer))): # дясна ръка
                        stopped = True
                        steer = False
                        print("S")
                        if bt:
                          arduino.write(bytes("s", 'utf-8'))
                      # Движение напред/назад с отворена ръка
                      elif(tilt > -40 and tilt < 15 and fingers[1] and fingers[2] and fingers[3] and fingers[4] and stopped or (tilt > -40 and tilt < 15 and steer)):
                        stopped = False
                        steer = False
                        if(not fingers[0]): # Палец свит -> Напред
                          print("F")
                          if bt:
                            arduino.write(bytes("F", 'utf-8'))
                        else: # Палец изпънат -> Назад
                          print("B")
                          if bt:
                            arduino.write(bytes("B", 'utf-8'))
                      # Аналогово завиване наляво
                      elif(tilt > tiltNegativeMin and tilt < tiltNegativeMax and refreshes >= 50): # Изпраща само на всеки 50 кадъра
                        tiltPLast = tiltP
                        tiltP = np.interp(tilt, [tiltNegativeMin, tiltNegativeMax], [-220, -75]) # Интерполиране на стойност за серво/мотор
                        stopped = False # Може да се движи и завива?
                        print(tiltP)
                        steer = True
                        # Изпраща стойност само ако промяната не е твърде голяма (филтър за трептене)
                        if bt and abs(tiltP - tiltPLast) < 15:
                          arduino.write(bytes(str(int(tiltP)) + "K", 'utf-8')) # 'K' за аналогово управление
                      # Аналогово завиване надясно
                      elif(tilt > tiltPositiveMin and tilt < tiltPositiveMax and refreshes >= 50):
                        tiltPLast = tiltP
                        tiltP = np.interp(tilt, [tiltPositiveMin, tiltPositiveMax], [75, 220])
                        stopped = False
                        print(tiltP)
                        steer = True
                        if bt and abs(tiltP - tiltPLast) < 15:
                          arduino.write(bytes(str(int(tiltP)) + "K", 'utf-8'))

                    elif(controlMode == 5): # може да се движи до определен ъгъл (изпраща ъгъл)
                      # Спиране с юмрук
                      if(tilt > tiltNegativeMax and tilt < tiltPositiveMin and not fingers[1] and not fingers[2] and not fingers[3] and not fingers[4] and (stopped == False or (stopped and steer))): # дясна ръка
                        stopped = True
                        steer = False
                        print("S")
                        if bt:
                          arduino.write(bytes("s", 'utf-8'))
                      # Движение напред/назад с отворена ръка
                      elif(tilt > -40 and tilt < 15 and fingers[1] and fingers[2] and fingers[3] and fingers[4] and stopped or (tilt > -40 and tilt < 15 and steer)):
                        stopped = False
                        steer = False
                        if(not fingers[0]): # Палец свит -> Напред
                          print("F")
                          if bt:
                            arduino.write(bytes("F", 'utf-8'))
                        else: # Палец изпънат -> Назад
                          print("B")
                          if bt:
                            arduino.write(bytes("B", 'utf-8'))
                      # Завиване наляво до определен ъгъл
                      elif(tilt > tiltNegativeMin and tilt < tiltNegativeMax and refreshes >= 50):
                        stopped = False
                        steer = True
                        # Изпраща ъгъла само ако промяната не е твърде голяма
                        if bt and abs(tilt - tiltLast) < 15:
                          arduino.write(bytes(str(int(tilt)) + "A", 'utf-8')) # 'A' за управление по ъгъл
                      # Завиване надясно до определен ъгъл
                      elif(tilt > tiltPositiveMin and tilt < tiltPositiveMax and refreshes >= 50):
                        stopped = False
                        steer = True
                        if bt and abs(tilt - tiltLast) < 15:
                          arduino.write(bytes(str(int(tilt)) + "A", 'utf-8'))

                    elif(controlMode == 6): # Не работи (почти безполезно) - изглежда като комбинация от 1 и 5
                      if(tilt > -40 and tilt < 20 and not fingers[1] and not fingers[2] and not fingers[3] and not fingers[4] and (stopped == False or (stopped and steer))): # дясна ръка
                        # arduino.write(bytes(str(tilt), 'utf-8')) # коментар
                        stopped = True
                        steer = False
                        print("S")
                        if bt:
                          arduino.write(bytes("s", 'utf-8'))
                      elif(tilt > -40 and tilt < 20 and stopped and fingers[1] and fingers[2] and fingers[3] and fingers[4]): # Отворена ръка
                        stopped = False
                        steer = False
                        print("F")
                        if bt:
                          arduino.write(bytes("F", 'utf-8'))
                      # Завиване по ъгъл
                      elif(tilt > tiltNegativeMin and tilt < tiltNegativeMax and refreshes >= 50):
                        stopped = False
                        steer = True
                        print(tilt)
                        if bt and abs(tilt - tiltLast) < 15:
                          arduino.write(bytes(str(int(tilt)) + "A", 'utf-8'))
                      elif(tilt > tiltPositiveMin and tilt < tiltPositiveMax and refreshes >= 50):
                        stopped = False
                        steer = True
                        print(tilt)
                        if bt and abs(tilt - tiltLast) < 15:
                          arduino.write(bytes(str(int(tilt)) + "A", 'utf-8'))
                      # Движение назад
                      elif(not fingers[1] and fingers[2] and fingers[3] and fingers[4] and stopped): # Само показалец свит
                        stopped = False
                        print("B")
                        if bt:
                          arduino.write(bytes("B", 'utf-8'))

                    elif(controlMode == 7): # Юмрук за спиране, ръка напред за движение напред, наклон за завиване (аналогово)
                      # Спиране (Юмрук - показалецът е по-ниско от основата си)
                      # if(tilt > -40 and tilt < 20 and not fingers[1] and not fingers[2] and not fingers[3] and not fingers[4] and (stopped == False or (stopped and steer))): # дясна ръка (старо условие)
                      if(pos6[1] < pos5[1] and fingers[1] and (not stopped or steer) and (not (tilt > tiltNegativeMin and tilt < tiltNegativeMax) and not (tilt > tiltPositiveMin and tilt < tiltPositiveMax))): # дясна ръка (пръстът е свит)
                        # arduino.write(bytes(str(tilt), 'utf-8')) # коментар
                        stopped = True # Спри
                        steer = False # Не завивай
                        print("S")
                        if bt:
                          arduino.write(bytes("s", 'utf-8')) # Изпрати "s"
                      # Движение напред (Ръка отворена - показалецът е по-високо от основата си)
                      # elif(tilt > -40 and tilt < 20 and stopped and fingers[1] and fingers[2] and fingers[3] and fingers[4]): # Старо условие
                      elif(pos6[1] > pos5[1] and (stopped or steer) and (not (tilt > tiltNegativeMin and tilt < tiltNegativeMax) and not (tilt > tiltPositiveMin and tilt < tiltPositiveMax))): # Пръстът е изпънат
                        stopped = False # Движи се
                        steer = False # Не завивай
                        print("F")
                        if bt:
                          arduino.write(bytes("F", 'utf-8')) # Изпрати "F"
                      # Аналогово завиване наляво
                      elif(tilt > tiltNegativeMin and tilt < tiltNegativeMax and refreshes >= 50): # Изпраща само на всеки 50 кадъра
                        # stopped = False # Не спира докато завива?
                        steer = True # Завивай
                        tiltPLast = tiltP
                        tiltP = np.interp(tilt, [tiltNegativeMin, tiltNegativeMax], [-220, -75]) # Интерполиране
                        print("left")
                        # if bt and abs(tilt - tiltLast) < 15: # Филтър (стар)
                        if bt:
                          # arduino.write(bytes(str(int(tilt)) + "A", 'utf-8')) # Управление по ъгъл (старо)
                          arduino.write(bytes(str(int(tiltP)) + "K", 'utf-8')) # Аналогово управление
                      # Аналогово завиване надясно
                      elif(tilt > tiltPositiveMin and tilt < tiltPositiveMax and refreshes >= 50):
                        # stopped = False # Не спира докато завива?
                        steer = True # Завивай
                        tiltPLast = tiltP
                        tiltP = np.interp(tilt, [tiltPositiveMin, tiltPositiveMax], [75, 220]) # Интерполиране
                        print("right")
                        # if bt and abs(tilt - tiltLast) < 15: # Филтър (стар)
                        if bt:
                          # arduino.write(bytes(str(int(tilt)) + "A", 'utf-8')) # Управление по ъгъл (старо)
                          arduino.write(bytes(str(int(tiltP)) + "K", 'utf-8')) # Аналогово управление
                      # Движение назад (показалец свит, другите изпънати - вероятно грешно условие)
                      elif(pos6[1] < pos5[1] and not fingers[1] and fingers[2] and fingers[3] and fingers[4] and stopped): # Само показалец свит
                        stopped = False
                        print("B")
                        if bt:
                          arduino.write(bytes("B", 'utf-8'))

                # Форматиране на състоянието на пръстите за показване
                finger1 = str(fingers[0]) + ", "
                finger2 = str(fingers[1]) + ", "
                finger3 = str(fingers[2]) + ", "
                finger4 = str(fingers[3]) + ", "
                finger5 = str(fingers[4])
                # Показване на състоянието на пръстите (ако devMode е активен)
                if devMode:
                  if not handsOnlyMode:
                    cv2.putText(frame1, "(" + finger1 + finger2 + finger3 + finger4 + finger5 + ")", (xRes-200, yRes-50), font, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
                  else:
                    cv2.putText(frame2, "(" + finger1 + finger2 + finger3 + finger4 + finger5 + ")", (xRes-200, yRes-50), font, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
        else: # Ако не са открити ръце
          # Аварийно спиране, ако Bluetooth е активен и превозното средство се движи
          if bt and not stopped:
            arduino.write(bytes("s", 'utf-8')) # Изпрати "s" (stop)
            stopped = True # Актуализирай състоянието
            print("emergency stop") # аварийно спиране

        # Показване на кадъра
        if not handsOnlyMode:
          cv2.imshow("Hand detection", frame1) # Показване на оригиналния кадър с наслагвания
        else:
          cv2.imshow("Hands only", frame2) # Показване само на ръцете върху черен фон

        # Обработка на натиснати клавиши
        key = cv2.waitKey(1) & 0xFF

        # Изход от програмата при натискане на 'q'
        if key == ord("q"):
            if tcp: # Затваряне на TCP сокета, ако е отворен
              client_socket.close()
            print("exited")
            if bt: # Спиране на превозното средство преди изход
              arduino.write(bytes("s", 'utf-8'))
            break # Прекъсване на цикъла

        # Отпечатване на стойности за отстраняване на грешки при натискане на 't'
        if key == ord("t"):
          print(treashhold) # извеждане на treashhold
          print(tilt) # извеждане на tilt
          print(tiltClosed) # извеждане на tiltClosed

        # Отпечатване на други стойности за отстраняване на грешки при натискане на 'z'
        if key == ord("z"):
          # print(abs(pos4[1] - pos8[1] - treashhold)) # извеждане на разстоянието за щипване спрямо прага
          print(pos0[0] - pos10[0]) # извеждане на X разлика 0-10
          print(pos0[1] - pos10[1]) # извеждане на Y разлика 0-10

        # Превключване на режими на управление с клавиши '1' до '9'
        if ord("1") <= key <= ord("9"):
          controlMode = key - 48 # Преобразуване на ASCII кода на цифрата в число
          print(f"Control mode switched to: {controlMode}") # извеждане на controlMode

        # Активиране/деактивиране на Bluetooth управлението с интервал (Space)
        if key == 32: # ASCII код за интервал
          if disabled: # Ако е било деактивирано
            print("Enabled BT control")
            disabled = False # Активирай
            # bt = True # Не променя bt тук, bt се управлява в началото
          else: # Ако е било активно
            print("Disabled BT control")
            if bt: # Изпрати команда за спиране при деактивиране
                arduino.write(bytes("s", 'utf-8'))
                stopped = True
            disabled = True # Деактивирай
            # bt = False # Не променя bt тук

# Освобождаване на камерата и затваряне на всички прозорци на OpenCV
cap.release()
cv2.destroyAllWindows()
if tcp: # Затваряне на сокета, ако е бил отворен (за всеки случай)
    client_socket.close()
if bt: # Затваряне на серийния порт, ако е бил отворен
    arduino.close()