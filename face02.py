import cv2
import mediapipe as mp
import numpy as np
import serial

import threading
import time
from pygame import mixer
from tkinter import *
from tkinter import ttk
from tkinter import messagebox
import serial.tools.list_ports
from imutils.video import VideoStream
import paho.mqtt.client as mqtt
import random
# from Adafruit_IO import Client, Feed, MQTTClient
import paho.mqtt.client as mqtt
#phan lua chon com
COM_USE = 0 
diaChiIP = 0
COM_XU = 0
numCoinOld = None 
numCoinCheck = None
def get_ports():
    
    ports = serial.tools.list_ports.comports()
    
    return ports

def findArduino(portsFound):
    listD = []
    commPort = 'None'
    numConnection = len(portsFound)
    
    for i in range(0,numConnection):
        port = foundPorts[i]
        strPort = str(port)
        splitPort = strPort.split(' ')
        listD.append(splitPort)

    return listD
            
                    
foundPorts = get_ports()        
connectPort = findArduino(foundPorts)
COM_VALUE = []
for i in range(0,len(foundPorts)):
    COM_VALUE.append(connectPort[i][0])


root = Tk()

root.geometry("450x300")
root.title("Lua chon COM")
ttk.Label(root, text = "Cong Quay :       ", 
          font = ("Times New Roman", 12)).grid(column = 0, 
          row = 0, padx = 10, pady = 10)
ttk.Label(root, text = "Cong Du phong :       ", 
          font = ("Times New Roman", 12)).grid(column = 0, 
          row = 1, padx = 10, pady = 10)
cmb = ttk.Combobox(root, width="10", values=COM_VALUE)#(COM_VALUE[0],COM_VALUE[1])
cmb2 = ttk.Combobox(root, width="10", values=COM_VALUE)
# dataTextBox = StringVar()
ttk.Label(root, text = "Nhap 'x' (192.168.x - dia chi ip):       ", 
          font = ("Times New Roman", 12)).grid(column = 0, 
          row = 3, padx = 10, pady = 10)
textBox = Entry(root, font=("Times New Roman", 12))
textBox.grid(row = 3, column = 10, pady = 10)
#add xu du phong
ttk.Label(root, text = "Nhập vào số COIN nếu có: ",
         font = ("Times New Roman", 12)).grid(column = 0, 
          row = 4, padx = 10, pady = 10)
textBoxCoin = Entry(root, font=("Times New Roman", 12))
textBoxCoin.grid(row = 4, column = 10, pady = 10)

def checkcmbo():
    global COM_USE, diaChiIP, COM_XU, numCoinOld, numCoinCheck
    for i in range(0,len(COM_VALUE)):
        if cmb.get() == cmb2.get():
            messagebox.showinfo("Lỗi rồi bạn ơi!!!","Không được chọn cổng COM giống nhau")
            break
        elif cmb.get() is not None:
            if textBox.get() == "":
                messagebox.showinfo("Lỗi rồi bạn ơi!!!","Vui lòng nhập địa chỉ IP của Camera")
                break
            elif textBox.get() is not None:
                # messagebox.showinfo("Thành công","----DONE----" + cmb.get() +"-"+ dataTextBox.get())
                COM_USE = cmb.get()
                COM_XU = cmb2.get()
                diaChiIP = textBox.get()
                numCoinOld = textBoxCoin.get()
                if len(numCoinOld) > 4:
                    messagebox.showinfo("Lỗi rồi bạn ơi", "Số coin <= 9999")
                else:
                    numCoinCheck = numCoinOld
                    numCoinOld = "Q"+numCoinOld + "P"
                    try:
                        if int(diaChiIP) >= 48 or int(diaChiIP)<=57:
                            
                            root.destroy()
                    except :
                        messagebox.showinfo("Lỗi rồi bạn ơi", "Địa chỉ ip phải dạng số")
                
            break


cmb.grid(row =0, column= 10, pady = 10)
cmb2.grid(row =1, column= 10, pady = 10)

btn = ttk.Button(root, text="OK",command=checkcmbo)
btn.grid(row =5, column= 10, pady = 10)

root.mainloop()

#===========================
# print("COM ", COM_USE)
ip = 'rtsp://admin:Duongtruong@192.168.1.233/Streaming/Channels/2'
ip_moi = 'rtsp://admin:Duongtruong@192.168.' + diaChiIP + '.233/Streaming/Channels/2'

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# cam = VideoStream(src = ip_moi, usePiCamera=False).start()
cam = VideoStream(src = 0, usePiCamera=False).start()
# ser = serial.Serial(COM_USE,9600, timeout=1)
# uartXu= serial.Serial(COM_XU,9600, timeout=1) 
global conLeft, conRight, conBet
conLeft = 0
conRight = 0
conBet = 0
tempp = 0
step =0
class handDetector():
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, modelComplexity=1, trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.modelComplex = modelComplexity
        self.trackCon = trackCon
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.modelComplex, 
                                        self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        # print(results.multi_hand_landmarks)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)
        return img
    def findPosition(self, img, handNo=0, draw=True):
        lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                # print(id, lm)
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                # print(id, cx, cy)
                lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 5, (255, 0, 255), cv2.FILLED)
        return lmList
song1 = ("quayTrai2.mp3")
song10 = ("quayTrai1.mp3")
song2 = ("quayPhai2.mp3")
song20 = ("quayPhai1.mp3")
song3 = ("TayDuKy.mp3")
song4 = ("ChungTaSauNay.mp3")
song = song3
song5 = ("nhinThang2.mp3")
song50 = ("nhinThang1.mp3")
song6 = ("DuongDen.mp3")
songChao = "xinChao.mp3"
def phatNhac():
    global tempp
    if tempp == 1:      
        mixer.init()
        mixer.music.load(song1)
        mixer.music.play()
        tempp = 0
    elif tempp == 10:
        mixer.init()
        mixer.music.load(song10)
        mixer.music.play()
        tempp = 0
    elif tempp == 2:
        mixer.init()
        mixer.music.load(song2)
        mixer.music.play()
        tempp = 0
    elif tempp == 20:
        mixer.init()
        mixer.music.load(song20)
        mixer.music.play()
        tempp = 0
    elif tempp == 3:
        mixer.init()
        mixer.music.load(song)
        mixer.music.play()
        tempp = 0
    elif tempp == 4:
        mixer.init()
        mixer.music.load(song5)
        mixer.music.play()
        tempp = 0
    elif tempp == 40:
        mixer.init()
        mixer.music.load(song50)
        mixer.music.play()
        tempp = 0
    elif tempp == 5:
        mixer.init()
        mixer.music.load(songChao)
        mixer.music.play()
        tempp = 0
# =============chuong trinh MQTT message ============

global dataRec
dataRec = 0
HOST = "ngoinhaiot.com"
PORT = 1111
USERNAME = "duong17"
PASSWORD = "BCD359D840744444"
mqttclient = mqtt.Client()


topQuayP = "qPhai"


FEED_ID = 'Counter'
conQuay = "duong17/conCong/conQuay"#"conQuay"
ngoaiVi = "duong17/conCong/ngoaiVi"

global dataRecOld, dataRecTemp
dataRecOld, dataRecTemp = 0, 0

def on_mqtt_connect():
    mqttclient.username_pw_set(USERNAME,PASSWORD)
    mqttclient.connect(HOST,PORT,60)
    mqttclient.loop_start()
def on_publish(topic, payload, qos):
    mqttclient.publish(topic, payload, qos)

def on_message_come(client, userdata, msg):
    # print(msg.topic + "." +":" + str(msg.payload))
    # print(type(msg))
    global dataRec, dataRecOld, dataRecTemp
    dataRecOld = (msg.payload)
    # print(type(msg.payload), "-", type(msg.topic))
    if msg.topic == ngoaiVi and dataRecOld != dataRec:
        dataRecTemp = dataRec = msg.payload
def on_subscribe():
    mqttclient.subscribe(ngoaiVi, 0)
    mqttclient.on_message = on_message_come

# Define callback functions which will be called when certain events happen.
# def connected(client):
#     """Connected function will be called when the client is connected to
#     Adafruit IO.This is a good place to subscribe to feed changes.  The client
#     parameter passed to this function is the Adafruit IO MQTT client so you
#     can make calls against it easily.
#     """
#     # Subscribe to changes on a feed named Counter.
#     print('Subscribing to Feed {0}'.format(FEED_ID))
#     client.subscribe(FEED_ID)
#     client.subscribe(ngoaiVi)
#     print('Waiting for feed data...')

# def disconnected(client):
#     """Disconnected function will be called when the client disconnects."""
#     sys.exit()

# def message(client, feed_id, payload):
#     global dataRec, dataRecOld, dataRecTemp
#     """Message function will be called when a subscribed feed has a new value.
#     The feed_id parameter identifies the feed, and the payload parameter has
#     the new value.
#     """
#     print('Feed {0} received new value: {1}'.format(feed_id, payload))
#     dataRecOld = payload
#     if feed_id == ngoaiVi and dataRecOld != dataRec:
#         dataRecTemp = dataRec = payload
        # client.publish(ngoaiVi,"demooooo")
        


# Create an MQTT client instance.
# client = MQTTClient(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)


#----------------------mqtt

#=======================#
global uart, viTriHienTai, numberCoin
global phanHoi, time30s, dieuKienDocXu, dieuKienRun, tempTime30s, runMode
global trangThaiMotor, state, tempBanDau
global tempLeft, tempRight, tempBet
global numTrai, numPhai, numBet, phatXongLan2, countRelay
global time1p
time1p = 0
numTrai = numPhai = numBet = phatXongLan2 = 0
countRelay = 0
(tempLeft, tempRight, tempBet) = (0, 0, 0)
trangThaiMotor = 0
phanHoi = 0
viTriHienTai = 0
runMode = 0
state, tempBanDau= None, None
readXu , numberCoin, time30s, dieuKienDocXu, dieuKienRun, tempTime30s = 0, 0, 0, 0, 0, 0
uart = True
runNhac = 0
#ngoai vi mqtt
def chuongTrinhNgoaiVi():
    global uart, conLeft, conRight, conBet, dataRec, runNhac
    global phanHoi, tempp, song, time3s, readXu, dieuKienDocXu, runMode
    global trangThaiMotor, viTriHienTai, numberCoin, dieuKienRun, tempBanDau
    
    while 1:
        # print("aajejek")
        # time.sleep(2)
        if dataRec == b'a\n' and runNhac == 0:#ben trai
            #ser.write(b'z*') # bao nhan ve goc
            # client.publish(ngoaiVi, "z*")
            on_publish(conQuay, "z*", 0)
            trangThaiMotor = 1 
            conRight = 0
            conBet = 0
            # chuong trinh phat nhac
            tempp = 3 
            song = song3
            phatNhac()
            runNhac = 1
        elif dataRec == b'b\n' and runNhac == 0:
            trangThaiMotor = 2
            conLeft = 0
            conBet = 0
            # chuong trinh phat nhac
            tempp = 3
            song = song4
            phatNhac()
            runNhac = 1
        elif dataRec == b'c\n' and runNhac == 0:
            trangThaiMotor = 3
            conLeft = conRight = 0
            #chuong trinh phat nhac
            tempp = 3
            song = song6
            phatNhac()
            runNhac = 1
        elif dataRec == b't\n': # ct trai
            #ser.write(b'o*')
            # client.publish(conQuay, "o*")
            on_publish(conQuay, "o*", 0)
            tempBanDau = 1 # bien ban dau xac dinh vi tri
            if trangThaiMotor == 1:
                trangThaiMotor = 0
                mixer.init()
                mixer.music.load(song3)
                mixer.music.stop()
                viTriHienTai = 1
            dataRec = 0
        elif dataRec == b'p\n': # ctht phai
            #ser.write(b'k*')
            # client.publish(conQuay, "k*")
            on_publish(conQuay, "k*", 0)
            if trangThaiMotor == 2:
                trangThaiMotor = 0
                mixer.init()
                mixer.music.load(song4)
                mixer.music.stop()  
                viTriHienTai = 2
                # runNhac = 0
            dataRec = 0
        elif dataRec == b'q\n': # ctht giua
            #ser.write(b'm*')
            # client.publish(conQuay, "m*")
            on_publish(conQuay, "m*", 0)
            if trangThaiMotor == 3:
                trangThaiMotor = 0
                mixer.init()
                mixer.music.load(song6)
                mixer.music.stop()   
                viTriHienTai = 3
                # runNhac = 0
            dataRec = 0
        elif dataRec == b'$\n':
            #ser.write(b'W*')
            # client.publish(ngoaiVi, "W*")
            on_publish(conQuay, "W*", 0)
            tempBanDau = 1    
            
t1 = threading.Thread(target=chuongTrinhNgoaiVi)# phuc vu UART
t1.daemon = True
t1.start() 

# ------------- -------------
# time3s = 0


detector = handDetector(detectionCon=0.75)           
pTime = 0
timeOUT = 0
tipIds = [4, 8, 12, 16, 20]
tempBanDau = None

# time.sleep(2)
# ser.write(b'$*')

# set Coin trong truong hop loi
# if numCoinOld != None and numCoinCheck != '' and int(numCoinCheck) != 0:
#     while tempBanDau == None:
#         pass
#     while int(numCoinCheck) != (numberCoin):
#         uartXu.write(bytes(numCoinOld, encoding='utf8'))
#         time.sleep(0.5)

#them xu tro lai khi loi trong qua trinh play

# bien chuong trinh chuyen trang thai

#============================================
# ham dieu khien dong co
def dieuKienRunDongCo():
    global numberCoin, dieuKienDocXu, time30s, time3s, runMode, tempTime30s
    if numberCoin == 0: dieuKienDocXu = 1
    else: dieuKienDocXu = 0
    #==================================================================#
    if time.time() - time30s >= 30 and tempTime30s == 0:
        if runMode == 0:
            tempTime30s = 1
            print("chay xong11")
            numberCoin = 0 # kiem tra lai so xu
        else:
            if readXu != b'o\n' : 
                uartXu.write(b's')
                print("lau")
            else:
                uartXu.write(b'S')
                runMode = 0
                tempTime30s = 1
                print("chay xong")
                time3s = time.time() # kiem tra xu tiep theo
                numberCoin = 0 # kiem tra lai so xu
    if tempTime30s == 1 and numberCoin > 0:
        runMode = 1
        tempTime30s = 0
        time30s = time.time()
        print("start runnnnn", numberCoin)

def xuLyGocNghieng():
    global  step, runMode, state, runNhac
    global tempRight, tempLeft, tempBet, conRight, conBet, conLeft, viTriHienTai, trangThaiMotor
    global timeOUT, cTime, pTime, text, tempp, time1p, tempBanDau
    global numBet, numTrai, numPhai, phatXongLan2, countRelay, dataRec, dataRecTemp
    runMode = 1
    try:
        #---------------------------------------------------#
        image1 = cam.read()
        image_old = image1

        if runMode == 1:
            image = image_old[0:360,160:480] #chia vung tac dong
        else: image = image_old
        image_old = cv2.flip(image_old, 1)
        if step == 0:  
            # buoc 1: nhan len goc nghieng cua mat
            # Flip the image horizontally for a later selfie-view display
            # Also convert the color space from BGR to RGB
            image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)

            # To improve performance
            image.flags.writeable = False
            
            # Get the result
            results = face_mesh.process(image)
            
            # To improve performance
            image.flags.writeable = True
            
            # Convert the color space from RGB to BGR
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            img_h, img_w, img_c = image.shape
            face_3d = []
            face_2d = []
            #dung phat nhac khi co xu moi vao
            if state != 1 and runMode == 1: 
                mixer.init()
                mixer.music.load(songChao)
                mixer.music.stop()
            if results.multi_face_landmarks:
                for face_landmarks in results.multi_face_landmarks:
                    for idx, lm in enumerate(face_landmarks.landmark):
                        if idx == 33 or idx == 263 or idx == 1 or idx == 61 or idx == 291 or idx == 199:
                            if idx == 1:
                                nose_2d = (lm.x * img_w, lm.y * img_h)
                                nose_3d = (lm.x * img_w, lm.y * img_h, lm.z * 8000)

                            x, y = int(lm.x * img_w), int(lm.y * img_h)

                            # Get the 2D Coordinates
                            face_2d.append([x, y])

                            # Get the 3D Coordinates
                            face_3d.append([x, y, lm.z])       
                    
                    # Convert it to the NumPy array
                    face_2d = np.array(face_2d, dtype=np.float64)

                    # Convert it to the NumPy array
                    face_3d = np.array(face_3d, dtype=np.float64)

                    # The camera matrix
                    focal_length = 1 * img_w

                    cam_matrix = np.array([ [focal_length, 0, img_h / 2],
                                            [0, focal_length, img_w / 2],
                                            [0, 0, 1]])

                    # The Distance Matrix
                    dist_matrix = np.zeros((4, 1), dtype=np.float64)

                    # Solve PnP
                    success, rot_vec, trans_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, dist_matrix)

                    # Get rotational matrix
                    rmat, jac = cv2.Rodrigues(rot_vec)

                    # Get angles
                    angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)

                    # Get the y rotation degree
                    x = angles[0] * 360
                    y = angles[1] * 360

                    # print(y)

                    # See where the user's head tilting
                    # trangThaiMotor: 1 - ben trai, 2- ben phai, 3 - giua

                    if runMode == 1:
                        state = 1
                        if y < -10:
                            text = "Nghieng ben trai"
                            tempRight = tempBet = 0
                            tempLeft += 1
                            
                            if (conLeft == 0 and trangThaiMotor == 0 and viTriHienTai !=1):
                                if tempLeft >= 50:
                                    tempLeft = 0
                                    for i in range(2):
                                        if i == 1: 
                                            numTrai += 1
                                            numBet = numPhai = 0
                                            if numTrai >2 : numTrai = 1

                                            conLeft = 1
                                            if numTrai == 2: tempp = 1
                                            elif numTrai == 1: tempp = 10
                                            phatNhac()
                                            step = 1 # tat dieu kien uart
                                            timeOUT = time.time()
                                            conBet = conRight = 0 #them

                                            
                        elif y > 10:
                            text = "Nghieng ben phai"
                            tempLeft = tempBet = 0
                            tempRight += 1
                            
                            if (conRight == 0 and trangThaiMotor == 0 and viTriHienTai !=2):
                                if tempRight >= 50: 
                                    # print("===== ben phai =====")
                                    tempRight = 0
                                    for i in range(2):
                                        
                                        if i == 1: 
                                            numPhai += 1
                                            numTrai = numBet = 0
                                            if numPhai > 2: numPhai = 1

                                            conRight = 1
                                            if numPhai == 2: tempp = 2
                                            elif numPhai == 1: tempp = 20
                                            phatNhac()
                                            step = 1 # tat dieu kien uart
                                            timeOUT = time.time()
                                            conBet = conLeft = 0 # them
                                        
                        elif x < -10:
                            text = "Nhin xuong"
                            tempRight = tempLeft = tempBet = 0
                        else:
                            text = "Nhin thang"
                            tempLeft = tempRight = 0
                            tempBet += 1
                            
                            if (conBet == 0 and trangThaiMotor == 0 and viTriHienTai!=3):
                                if tempBet >= 50:
                                    # print("==== nhin thang-----------")
                                    tempBet = 0
                                    for i in range(2):
                                        
                                        if i == 1: 
                                            numBet += 1
                                            numPhai = numTrai = 0
                                            if numBet >2: numBet = 1

                                            conBet = 1
                                            if numBet == 2: tempp = 4 # o giua
                                            elif numBet == 1: tempp = 40
                                            phatNhac()
                                            step = 1 # tat dieu kien uart
                                            timeOUT = time.time()
                                            conRight = conLeft = 0
                            

                        # Display the nose direction
                        nose_3d_projection, jacobian = cv2.projectPoints(nose_3d, rot_vec, trans_vec, cam_matrix, dist_matrix)

                        p1 = (int(nose_2d[0]), int(nose_2d[1]))
                        p2 = (int(nose_3d_projection[0][0][0]), int(nose_3d_projection[0][0][1]))
                        
                        # cv2.line(image_old, p1, p2, (255, 0, 0), 2)# tat sau khi roi

                        # Add the text on the image
                        cv2.putText(image_old, text, (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    # chua nap xuu
                if runMode == 0 and tempBanDau == 1 and trangThaiMotor == 0:
                    if state == 1 or state == None:
                        time1p = time.time()
                        state = 2
                    if time.time()-time1p>= 10 and state == 2: # 10s ko co xu
                        state = 3
                    elif time.time()-time1p>= 20 and state == 4:# phat lai nhac
                        time1p = time.time()
                        tempp = 5
                        phatNhac()
                    if state == 3: # phat nhac
                        time1p = time.time()
                        tempp = 5
                        phatNhac()
                        state = 4
                    cv2.putText(image_old, "Chua co xu", (0, 20), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 255, 0), 2)

            # cv2.imshow('Head Pose Estimation', image)
        elif step == 1:
            if numBet != 2 and numPhai != 2 and numTrai != 2:
                if time.time() - timeOUT > 5:
                    conBet = conLeft = conRight = 0
                    step = 0
                    phatXongLan2 = 0
                    countRelay = 0
            else: 
                # cho lenh chay
                if time.time() - timeOUT > 3 and phatXongLan2 == 0:
                    phatXongLan2 = 1
                    
                if phatXongLan2 == 1:
                    countRelay += 1
                    # quay trai
                    if numTrai == 2:
                        if dataRec != b'a\n':
                            if dataRec == b't\n': #050122
                                step = 0
                                viTriHienTai = 1 
                                conBet = conRight = 0
                            else:
                                if countRelay >= 14:
                                    # ser.write(b'a*') # sua
                                    time.sleep(0.5)
                                    # client.publish(conQuay, "a*")
                                    on_publish(conQuay, "a*", 0)
                                    countRelay = 0
                            if dataRecTemp == b't\n': #20220425
                                step = 0
                                viTriHienTai = 1 
                                conBet = conRight = 0
                        elif dataRec == b'a\n':
                            step = 0
                            runNhac = 0
                    # quay phai
                    elif numPhai == 2:
                        if dataRec != b'b\n': #dataRec != b'b\n' or 
                            if dataRec == b'p\n': #050122
                                step = 0
                                viTriHienTai = 2
                                conBet = conLeft = 0
                            else: 
                                if countRelay >= 14:
                                    # ser.write(b'b*') # sua
                                    time.sleep(0.5)
                                    # client.publish(conQuay, "b*")
                                    on_publish(conQuay, "b*", 0)
                                    countRelay = 0
                            if dataRecTemp == b'p\n': #20220425
                                step = 0
                                viTriHienTai = 2
                                conBet = conLeft = 0
                        elif dataRec == b'b\n': #dataRec == b'b\n' or 
                            step = 0
                            runNhac = 0
                    # ve giua
                    elif numBet == 2:
                        if dataRec != b'c\n':
                            if dataRec == b'q\n': #050122
                                time.sleep(0.5)
                                step = 0
                                viTriHienTai = 3
                                conLeft = conRight = 0
                            else:
                                if countRelay >= 14:
                                    # ser.write(b'c*')
                                    time.sleep(0.5)
                                    # client.publish(conQuay, "c*")
                                    on_publish(conQuay, "c*", 0)
                                    countRelay = 0
                            if dataRecTemp == b'q\n': #them 20220425
                                step = 0
                                viTriHienTai = 3
                                conLeft = conRight = 0
                        elif dataRec == b'c\n':
                            step = 0
                            runNhac = 0
            
        cv2.rectangle(image_old, [160,0], [480,360], (0,0,255),3)
        cv2.imshow("Design by Duong Truong", image_old)
        # cv2.imshow("roi",image)
        
    except AttributeError:
        pass
    
# Setup the callback functions defined above.
#MQTT
# client.on_connect = connected
# client.on_disconnect = disconnected
# client.on_message = message

# # Connect to the Adafruit IO server.
# client.connect()

# client.loop_background()       
on_mqtt_connect()
on_subscribe()   
while True: #threaded_camera.capture.isOpened(): #if self.capture.isOpened(): #cap.isOpened()
    # dieuKienRunDongCo()
    
    xuLyGocNghieng()
   
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break

 
cam.stop()
cv2.destroyAllWindows()
# client.loop_stop()
# client.disconnect()
