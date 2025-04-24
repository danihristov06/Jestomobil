import threading
import time
import math
import mediapipe as mp
import cv2
import serial
import socket
import keyboard
import numpy as np

# Settings?
handsOnlyMode = True # Whether it should display only the hands or camera preview
numberOfHands = 1
cs = False
tcp = False  # Send data to steam driver
devMode = True # Toggles display when the fingers are up and shows the numbers of each point
bt = True # Whether it should try to connect to bluetooth or not
controlMode = 7 # Toggle with the numbers
disabled = False
xRes = 800 # 1280 800 640
yRes = 450 # 720 450 360
resMulti = 1.6 #1 1.6 2 if you change the resolution you should change this too based on the resolution to match 1280 always

# Advanced settings
base_scale = 100  # Base scaling factor
# depth_multiplier = 6.5  # Adjust this to fine-tune sensitivity
depth_multiplier = 60  # Adjust this to fine-tune sensitivity
tiltNegativeMin = -65 # -60 closed values -80 normal values
tiltNegativeMax = -20 # -25 -40
tiltPositiveMin= 25 # 25 15
tiltPositiveMax= 60 # 60 40

if bt:
  arduino = serial.Serial(port='COM12', baudrate=9600, timeout=.1) # 7

# Use MediaPipe to draw the hand framework over the top of hands it identifies in Real-Time
drawingModule = mp.solutions.drawing_utils
handsModule = mp.solutions.hands

# Use CV2 Functionality to create a Video stream and add some values
cap = cv2.VideoCapture(0)
# fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')

# Sending the data to c++ driver
if(tcp):
  client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  client_socket.connect(("localhost", 12345))

fingers = [0, 0, 0, 0, 0]
# clicked = {"Left": False, "Right": False}  # Tracks pinch state for each hand
clickedLeft = False
clickedRight = False
# treashhold = 65 # reference value

# Initialize variables for positions
pos0, pos1, pos2, pos3, pos4, pos5, pos6, pos8, pos9, pos10, pos12, pos13, pos14, pos16, pos17, pos18, pos20, prevPos, currentPos = [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]
handPosX, handPosY, refreshes, clicked, prev_frame_time, new_frame_time, fps, tilt, tiltP, tiltPLast, tiltLast, tiltClosed = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 # refreshes default 5
stopped, steer, wait = True, False, False
normal_vector = [0, 0, 0]
font = cv2.FONT_HERSHEY_SIMPLEX

def get_hand_type(hand_landmarks, width, height):
    if hand_landmarks.landmark[mp.solutions.hands.HandLandmark.WRIST].x * width < width / 2:
        return 'Left'
    else:
        return 'Right'
    
def detect_pinch(hand_landmarks, hand_type, pos4, pos8):
    global clickedLeft
    global clickedRight
    pos4 = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.THUMB_TIP]
    pos8 = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP]
    distance = math.sqrt((pos4.x - pos8.x) ** 2 + (pos4.y - pos8.y) ** 2)
    if(hand_type == "Left"):
      if distance < 0.075 and not clickedLeft:
          clickedLeft = True
          if tcp:
            client_socket.send(str("pl" + "\n").encode())  # Send as a string
          # print(f"{hand_type} hand pinch detected")
      elif distance >= 0.05 and clickedLeft:
          clickedLeft = False
          if tcp:
            client_socket.send(str("u" + "\n").encode())  # Send as a string
    elif(hand_type == "Right"):
      if distance < 0.05 and not clickedRight:
          clickedRight = True
          if tcp:
            client_socket.send(str("pr" + "\n").encode())  # Send as a string
          # print(f"{hand_type} hand pinch detected")
      elif distance >= 0.05 and clickedRight:
          clickedRight = False
          if tcp:
            client_socket.send(str("u" + "\n").encode())  # Send as a string

def sendPos(h, x, y, ry, rx, rz):
  client_socket.send(str("H" + h + "\n").encode())  # Send as a string
  client_socket.send(str("Y" + str(y) + "\n").encode())  # Send as a string
  client_socket.send(str("X" + str(abs(x - xRes/2) * resMulti) + "\n").encode())  # Send as a string
  client_socket.send(str("RX" + str(rx * -1) + "\n").encode())  # Send as a string
  client_socket.send(str("RY" + str(ry * -1) + "\n").encode())  # Send as a string
  client_socket.send(str("RZ" + str(rz) + "\n").encode())  # Send as a string

# Add confidence values and extra settings to MediaPipe hand tracking
with handsModule.Hands(static_image_mode=False, min_detection_confidence=0.7, min_tracking_confidence=0.5, max_num_hands=numberOfHands) as hands:
    while True:
        ret, frame = cap.read()

        # frame1 = cv2.flip(cv2.resize(frame, (xRes, yRes)), 1) #decrease for better performance
        frame1 = cv2.resize(frame, (xRes, yRes)) # not flipped
        frame2 = frame1 - frame1

        results = hands.process(cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)) #overlays the hands
        new_frame_time = time.time()
        fps = str(int(1/(new_frame_time-prev_frame_time)))
        prev_frame_time = new_frame_time

        if not handsOnlyMode:
            cv2.putText(frame1, fps, (0, 18), font, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            cv2.putText(frame2, fps, (0, 18), font, 0.75, (0, 255, 0), 2, cv2.LINE_AA)

        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                hand_type = handedness.classification[0].label
                # print(f"Hand type: {hand_type}")

                # Draw the hand landmarks on the frame
                drawingModule.draw_landmarks(frame1, hand_landmarks, handsModule.HAND_CONNECTIONS)
                drawingModule.draw_landmarks(frame2, hand_landmarks, handsModule.HAND_CONNECTIONS)

                # Calculate the center of the hand
                handPosX = int((hand_landmarks.landmark[0].x * xRes + hand_landmarks.landmark[5].x * xRes + hand_landmarks.landmark[9].x * xRes + hand_landmarks.landmark[13].x * xRes + hand_landmarks.landmark[17].x * xRes) / 5)
                handPosY = int((hand_landmarks.landmark[0].y * yRes + hand_landmarks.landmark[5].y * yRes + hand_landmarks.landmark[9].y * yRes + hand_landmarks.landmark[13].y * yRes + hand_landmarks.landmark[17].y * yRes) / 5)
                tiltZ = np.interp(tilt, [-120, 120], [-1, 1])
                # print(tiltZ)

                # Draw a blue dot for the left hand and a red dot for the right hand and send info
                if hand_type == 'Left':
                  if(tcp):
                    sendPos("L", handPosX, handPosY, normal_vector[0], normal_vector[1], tiltZ)
                  if not handsOnlyMode:
                    cv2.circle(frame1, (handPosX, handPosY), 4, (255, 0, 0), -1) # Blue dot
                  else:
                    cv2.circle(frame2, (handPosX, handPosY), 4, (255, 0, 0), -1) # Blue dot
                else:
                  if(tcp):
                    sendPos("R", handPosX, handPosY, normal_vector[0], normal_vector[1], tiltZ)
                  if not handsOnlyMode:
                    cv2.circle(frame1, (handPosX, handPosY), 4, (0, 0, 255), -1) # Red dot
                  else:
                    cv2.circle(frame2, (handPosX, handPosY), 4, (0, 0, 255), -1) # Red dot

                for point in handsModule.HandLandmark:
                  normalizedLandmark = hand_landmarks.landmark[point]
                  points = np.array([
                      [hand_landmarks.landmark[0].x, hand_landmarks.landmark[0].y, hand_landmarks.landmark[0].z],
                      [hand_landmarks.landmark[5].x, hand_landmarks.landmark[5].y, hand_landmarks.landmark[5].z],
                      [hand_landmarks.landmark[17].x, hand_landmarks.landmark[17].y, hand_landmarks.landmark[17].z]
                  ])
                  normal_vector = np.cross(points[2] - points[0], points[1] - points[2])
                  normal_vector /= np.linalg.norm(normal_vector)
                  # print(normal_vector)
                  pixelCoordinatesLandmark = drawingModule._normalized_to_pixel_coordinates(normalizedLandmark.x, normalizedLandmark.y, xRes, yRes)
                  if pixelCoordinatesLandmark is None:
                      break
                  # print(point.value, hand_landmarks.landmark[point.value])
                  if pixelCoordinatesLandmark and devMode:
                    # Display point number next to each landmark
                    cv2.putText(frame1, str(point.value), pixelCoordinatesLandmark, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(frame2, str(point.value), pixelCoordinatesLandmark, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                  # z_depth = abs((hand_landmarks.landmark[0].z + hand_landmarks.landmark[3].z + hand_landmarks.landmark[6].z + hand_landmarks.landmark[9].z + hand_landmarks.landmark[12].z + hand_landmarks.landmark[15].z + hand_landmarks.landmark[17].z + hand_landmarks.landmark[20].z) / 8) # Calculate the average Z depth
                  z_depth = abs(pos5[0] - pos0[0]) - abs(pos17[0] - pos0[0]) # 5-0 17-0
                  # cv2.circle(frame2, (z_depth, 400), 4, (255, 0, 0), -1)
                  # treashhold = int(float((-100 * float(hand_landmarks.landmark[8].z)) * 6.5))
                  treashhold = int(abs(z_depth * depth_multiplier / base_scale))


                  # Update positions for different points
                  # Side finger
                  if point == 0: pos0 = pixelCoordinatesLandmark
                  if point == 2: pos2 = pixelCoordinatesLandmark
                  if point == 3: pos3 = pixelCoordinatesLandmark
                  if point == 4: 
                    pos4 = pixelCoordinatesLandmark
                    if pos4[0] < pos3[0]:
                        fingers[0] = 1
                    else:
                        fingers[0] = 0
                  if point == 5: pos5 = pixelCoordinatesLandmark
                  # Pointer finger
                  if point == 6: pos6 = pixelCoordinatesLandmark
                  if point == 8:
                    pos8 = pixelCoordinatesLandmark
                    if pos8[1] > pos6[1]:
                      fingers[1] = 1
                    else:
                      fingers[1] = 0
                  # Controls the pc mouse and gestures for vr
                  detect_pinch(hand_landmarks, hand_type, pos4, pos8)

                    # if(cs): # Move mouse
                    #   win32api.mouse_event(win32con.MOUSEEVENTF_MOVE | win32con.MOUSEEVENTF_ABSOLUTE, int((0 + (currentPos[0] - prevPos[0]))/win32api.GetSystemMetrics(0)*11111), int((0 + (prevPos[1] - currentPos[1]))/win32api.GetSystemMetrics(1)*11111) ,0 ,0) # csgo settings
                    # else:
                    #   win32api.mouse_event(win32con.MOUSEEVENTF_MOVE | win32con.MOUSEEVENTF_ABSOLUTE, int((mousePos[0] + (prevPos[0] - handPosX))/win32api.GetSystemMetrics(0)*65555), int((mousePos[1] + (handPosY - prevPos[1]))/win32api.GetSystemMetrics(1)*65555) ,0 ,0) # normal use

                    # Update previous position
                    # prevPos = [handPosX, handPosY]
                            
                    # Left click
                    # if pos4[1] - pos8[1] < int(treashhold/2.5) and fingers[2] == 0 and fingers[3] == 0 and fingers[4] == 0:
                    #     mouse.press(button='left')
                    # elif pos4[1] - pos8[1] > int(treashhold/2.5) and fingers[2] == 0 and fingers[3] == 0 and fingers[4] == 0:  # 25 default treashold
                    #   mouse.release(button='left')
                  if point == 9: pos9 = pixelCoordinatesLandmark
                  # Middle finger
                  if point == 10: pos10 = pixelCoordinatesLandmark
                  if point == 12:
                    pos12 = pixelCoordinatesLandmark
                    if pos12[1] > pos10[1]:
                      fingers[2] = 1
                    else:
                      fingers[2] = 0
                  if point == 13: pos13 = pixelCoordinatesLandmark
                  # Fourth finger
                  if point == 14: pos14 = pixelCoordinatesLandmark
                  if point == 16: 
                    pos16 = pixelCoordinatesLandmark
                    if pos16[1] > pos14[1]:
                      fingers[3] = 1
                    else:
                      fingers[3] = 0
                  if point == 17: pos17 = pixelCoordinatesLandmark
                  # Smoll finger
                  if point == 18: pos18 = pixelCoordinatesLandmark
                  if point == 20: 
                    pos20 = pixelCoordinatesLandmark
                    if pos20[1] > pos18[1]:
                      fingers[4] = 1
                    else:
                      fingers[4] = 0
                  
                  if(refreshes == 50): # Normally used for mouse control now for any delays needed (like sending data)
                    refreshes = 0
                    wait = False
                    # currentPos = prevPos
                  elif(not refreshes == 50):
                    refreshes = refreshes + 1
                    # prevPos = [handPosX, handPosY]


                  tiltLast = tilt
                  tiltClosedLast = tiltClosed
                  # tilt = math.degrees(math.atan2(pos9[0] - pos12[0], pos9[1] - pos12[1])) # Calculate vector for the middle finger
                  # tilt = math.degrees(math.atan2(pos0[0] - pos9[0], pos0[1] - pos9[1])) # Calculate vector for the wrist
                  tilt = math.degrees(math.atan2(pos0[0] - (pos9[0] + pos13[0]) / 2, pos0[1] - (pos9[1] + pos13[1]) / 2)) # Calculate vector for the wrist to knuckle for fist control
                  tiltClosed = math.degrees(math.atan2(pos0[0] - (pos9[0] + pos13[0]) / 2, pos0[1] - (pos9[1] + pos13[1]) / 2))

                  # 0 gore 1 doly
                  if(bt):
                    if(controlMode == 1): # fist forward one finger up backward
                      if(tilt > -40 and tilt < 20 and not fingers[1] and not fingers[2] and not fingers[3] and not fingers[4] and (stopped == False or (stopped and steer))): # right hand
                        # arduino.write(bytes(str(tilt), 'utf-8'))
                      # if((fingers[1] and fingers[2] and fingers[3] and fingers[4]) == True and not stopped): # right hand
                        stopped = True
                        steer = False
                        print("S")
                        if bt:
                          arduino.write(bytes("s", 'utf-8'))
                      elif(tilt > -40 and tilt < 20 and stopped and fingers[1] and fingers[2] and fingers[3] and fingers[4]):
                        stopped = False
                        steer = False
                        print("F")
                        if bt:
                          arduino.write(bytes("F", 'utf-8'))
                      elif(tilt > 20 and not steer):
                        stopped = True
                        steer = True
                        print("R")
                        if bt:
                          arduino.write(bytes("R", 'utf-8'))
                      elif(tilt < -40 and not steer):
                        stopped = True
                        steer = True
                        print("L")
                        if bt:
                          arduino.write(bytes("L", 'utf-8'))

                      elif(not fingers[1] and fingers[2] and fingers[3] and fingers[4] and stopped):
                        stopped = False
                        print("B")
                        if bt:
                          arduino.write(bytes("B", 'utf-8'))
                    elif(controlMode == 2): # the reverse of the first option but a bit better (pretty much the same now)
                      if(tilt > -40 and tilt < 20 and not fingers[1] and not fingers[2] and not fingers[3] and not fingers[4] and stopped == False): # right hand
                        # arduino.write(bytes(str(tilt), 'utf-8'))
                      # if((fingers[1] and fingers[2] and fingers[3] and fingers[4]) == True and not stopped): # right hand
                        stopped = True
                        if bt:
                          arduino.write(bytes("s", 'utf-8'))
                      elif(tilt > 20):
                        stopped = False
                        print("R")
                        if bt:
                          arduino.write(bytes("R", 'utf-8'))
                      elif(tilt < -40):
                        stopped = False
                        print("L")
                        if bt:
                          arduino.write(bytes("L", 'utf-8'))

                      elif(not fingers[1] and  fingers[2] and fingers[3] and fingers[4] and stopped):
                        stopped = False
                        print("F")
                        if bt:
                          arduino.write(bytes("F", 'utf-8'))
                      elif(fingers[1] and fingers[2] and fingers[3] and not fingers[4] and stopped):
                        stopped = False
                        print("B")
                        if bt:
                          arduino.write(bytes("B", 'utf-8'))
                    elif(controlMode == 3): # fist with finger in front = back fist with finger outside = forward
                      if(tilt > -40 and tilt < 20 and stopped == False and not fingers[1] and not fingers[2] and not fingers[3] and not fingers[4]): # right hand
                        # arduino.write(bytes(str(tilt), 'utf-8'))
                      # if((fingers[1] and fingers[2] and fingers[3] and fingers[4]) == True and not stopped): # right hand
                        stopped = True
                        steer = False
                        print("S")
                        if bt:
                          arduino.write(bytes("s", 'utf-8'))
                      elif(tilt > -40 and tilt < 20 and fingers[1] and fingers[2] and fingers[3] and fingers[4] and stopped or (tilt > -40 and tilt < 20 and steer)):
                        stopped = False
                        steer = False
                        if(not fingers[0]):
                          print("F")
                          if bt:
                            arduino.write(bytes("F", 'utf-8'))
                        else:
                          print("B")
                          if bt:
                            arduino.write(bytes("B", 'utf-8'))
                      elif(tilt > 20 and not steer):
                        stopped = False
                        steer = True
                        print("R")
                        if bt:
                          arduino.write(bytes("R", 'utf-8'))
                      elif(tilt < -40 and not steer):
                        stopped = False
                        steer = True
                        print("L")
                        if bt:
                          arduino.write(bytes("L", 'utf-8'))
                    elif(controlMode == 4): # same as 3 but with analog tilt
                      if(tilt > tiltNegativeMax and tilt < tiltPositiveMin and not fingers[1] and not fingers[2] and not fingers[3] and not fingers[4] and (stopped == False or (stopped and steer))): # right hand
                        stopped = True
                        steer = False
                        print("S")
                        if bt:
                          arduino.write(bytes("s", 'utf-8'))
                      elif(tilt > -40 and tilt < 15 and fingers[1] and fingers[2] and fingers[3] and fingers[4] and stopped or (tilt > -40 and tilt < 15 and steer)):
                        stopped = False
                        steer = False
                        if(not fingers[0]):
                          print("F")
                          if bt:
                            arduino.write(bytes("F", 'utf-8'))
                        else:
                          print("B")
                          if bt:
                            arduino.write(bytes("B", 'utf-8'))
                      elif(tilt > tiltNegativeMin and tilt < tiltNegativeMax and refreshes >= 50):
                        tiltPLast = tiltP
                        tiltP = np.interp(tilt, [tiltNegativeMin, tiltNegativeMax], [-220, -75])
                        stopped = False
                        print(tiltP)
                        steer = True
                        if bt and abs(tiltP - tiltPLast) < 15:
                          arduino.write(bytes(str(int(tiltP)) + "K", 'utf-8'))
                      elif(tilt > tiltPositiveMin and tilt < tiltPositiveMax and refreshes >= 50):
                        tiltPLast = tiltP
                        tiltP = np.interp(tilt, [tiltPositiveMin, tiltPositiveMax], [75, 220])
                        stopped = False
                        print(tiltP)
                        steer = True
                        if bt and abs(tiltP - tiltPLast) < 15:
                          arduino.write(bytes(str(int(tiltP)) + "K", 'utf-8'))
                    elif(controlMode == 5): # can move to a specific angle 
                      if(tilt > tiltNegativeMax and tilt < tiltPositiveMin and not fingers[1] and not fingers[2] and not fingers[3] and not fingers[4] and (stopped == False or (stopped and steer))): # right hand
                        stopped = True
                        steer = False
                        print("S")
                        if bt:
                          arduino.write(bytes("s", 'utf-8'))
                      elif(tilt > -40 and tilt < 15 and fingers[1] and fingers[2] and fingers[3] and fingers[4] and stopped or (tilt > -40 and tilt < 15 and steer)):
                        stopped = False
                        steer = False
                        if(not fingers[0]):
                          print("F")
                          if bt:
                            arduino.write(bytes("F", 'utf-8'))
                        else:
                          print("B")
                          if bt:
                            arduino.write(bytes("B", 'utf-8'))
                      elif(tilt > tiltNegativeMin and tilt < tiltNegativeMax and refreshes >= 50):
                        stopped = False
                        steer = True
                        if bt and abs(tilt - tiltLast) < 15:
                          arduino.write(bytes(str(int(tilt)) + "A", 'utf-8'))
                      elif(tilt > tiltPositiveMin and tilt < tiltPositiveMax and refreshes >= 50):
                        stopped = False
                        steer = True
                        if bt and abs(tilt - tiltLast) < 15:
                          arduino.write(bytes(str(int(tilt)) + "A", 'utf-8'))
                    elif(controlMode == 6): # Doesn't work (pretty much useless)
                      if(tilt > -40 and tilt < 20 and not fingers[1] and not fingers[2] and not fingers[3] and not fingers[4] and (stopped == False or (stopped and steer))): # right hand
                        # arduino.write(bytes(str(tilt), 'utf-8'))
                      # if((fingers[1] and fingers[2] and fingers[3] and fingers[4]) == True and not stopped): # right hand
                        stopped = True
                        steer = False
                        print("S")
                        if bt:
                          arduino.write(bytes("s", 'utf-8'))
                      elif(tilt > -40 and tilt < 20 and stopped and fingers[1] and fingers[2] and fingers[3] and fingers[4]):
                        stopped = False
                        steer = False
                        print("F")
                        if bt:
                          arduino.write(bytes("F", 'utf-8'))
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

                      elif(not fingers[1] and fingers[2] and fingers[3] and fingers[4] and stopped):
                        stopped = False
                        print("B")
                        if bt:
                          arduino.write(bytes("B", 'utf-8'))
                    elif(controlMode == 7): # Fist to stop Hand forward for forward tilt for steering
                      # if(tilt > -40 and tilt < 20 and not fingers[1] and not fingers[2] and not fingers[3] and not fingers[4] and (stopped == False or (stopped and steer))): # right hand
                      if(pos6[1] < pos5[1] and fingers[1] and (not stopped or steer) and (not (tilt > tiltNegativeMin and tilt < tiltNegativeMax) and not (tilt > tiltPositiveMin and tilt < tiltPositiveMax))): # right hand
                        # arduino.write(bytes(str(tilt), 'utf-8'))
                      # if((fingers[1] and fingers[2] and fingers[3] and fingers[4]) == True and not stopped): # right hand
                        stopped = True
                        steer = False
                        print("S")
                        if bt:
                          arduino.write(bytes("s", 'utf-8'))
                      # elif(tilt > -40 and tilt < 20 and stopped and fingers[1] and fingers[2] and fingers[3] and fingers[4]):
                      elif(pos6[1] > pos5[1] and (stopped or steer) and (not (tilt > tiltNegativeMin and tilt < tiltNegativeMax) and not (tilt > tiltPositiveMin and tilt < tiltPositiveMax))):
                        stopped = False
                        steer = False
                        print("F")
                        if bt:
                          arduino.write(bytes("F", 'utf-8'))
                      elif(tilt > tiltNegativeMin and tilt < tiltNegativeMax and refreshes >= 50):
                        # stopped = False
                        steer = True
                        tiltPLast = tiltP
                        tiltP = np.interp(tilt, [tiltNegativeMin, tiltNegativeMax], [-220, -75])
                        print("left")
                        # if bt and abs(tilt - tiltLast) < 15:
                        if bt:
                          # arduino.write(bytes(str(int(tilt)) + "A", 'utf-8'))
                          arduino.write(bytes(str(int(tiltP)) + "K", 'utf-8'))
                      elif(tilt > tiltPositiveMin and tilt < tiltPositiveMax and refreshes >= 50):
                        # stopped = False
                        steer = True
                        tiltPLast = tiltP
                        tiltP = np.interp(tilt, [tiltPositiveMin, tiltPositiveMax], [75, 220])
                        print("right")
                        # if bt and abs(tilt - tiltLast) < 15:
                        if bt:
                          # arduino.write(bytes(str(int(tilt)) + "A", 'utf-8'))
                          arduino.write(bytes(str(int(tiltP)) + "K", 'utf-8'))

                      elif(pos6[1] < pos5[1] and not fingers[1] and fingers[2] and fingers[3] and fingers[4] and stopped):
                        stopped = False
                        print("B")
                        if bt:
                          arduino.write(bytes("B", 'utf-8'))

                    # elif(not fingers[1] and fingers[2] and fingers[3] and fingers[4] and stopped):
                    #   stopped = False
                    #   print("B")
                    #   if bt:
                    #     arduino.write(bytes("B", 'utf-8'))

                finger1 = str(fingers[0]) + ", "
                finger2 = str(fingers[1]) + ", "
                finger3 = str(fingers[2]) + ", "
                finger4 = str(fingers[3]) + ", "
                finger5 = str(fingers[4])
                if devMode:
                  if not handsOnlyMode:
                    cv2.putText(frame1, "(" + finger1 + finger2 + finger3 + finger4 + finger5 + ")", (600, 600), font, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
                  else:
                    cv2.putText(frame2, "(" + finger1 + finger2 + finger3 + finger4 + finger5 + ")", (600, 600), font, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
        else:
          if bt and not stopped:
            arduino.write(bytes("s", 'utf-8'))
            stopped = True
            print("emergency stop")
            
        if not handsOnlyMode:
          cv2.imshow("Hand detection", frame1)
        else:
          cv2.imshow("Hands only", frame2)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            if tcp:
              client_socket.close()
            print("exited")
            if bt:
              arduino.write(bytes("s", 'utf-8'))
            break
        if key == ord("t"):
          print(treashhold)
          print(tilt)
          print(tiltClosed)
        if key == ord("z"):
          # print(abs(pos4[1] - pos8[1] - treashhold))
          print(pos0[0] - pos10[0])
          print(pos0[1] - pos10[1])
        # Switch modes
        if ord("1") <= key <= ord("9"):
          controlMode = key - 48
          print(controlMode)
          
        # Enable disable
        if key == 32:
          if disabled:
            print("Disabled")
            disabled = False
            bt = False
          else:
            print("Enabled")
            bt = True
            disabled = True