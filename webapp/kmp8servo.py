from flask import Flask, render_template, request, jsonify, session, url_for, send_file, copy_current_request_context
from flask_session import Session
from flask_socketio import SocketIO, emit, join_room, leave_room, close_room, rooms, disconnect
from flask_executor import Executor

import eventlet
from eventlet import wsgi
from eventlet import websocket
from threading import Thread, Event, Lock

import math
import serial
import config

import requests
from datetime import datetime
import os.path
from glob import glob
import os
import atexit
import numpy as np
import json
import random

import traceback

import io
from time import strftime, sleep
import time

import picamera

async_mode = None
app = Flask(__name__)

SESSION_TYPE = 'filesystem'
app.config.from_object(__name__)
Session(app)

socketio = SocketIO(app, async_mode=async_mode)

thread = None
thread_lock = Lock()
#for stopping the robot between http requests
stop_robot_event = Event()



is_robot = config.settings['robot']
has_lidar = config.settings['lidar']
has_arduino_at_115200 = config.settings['arduino_115200']
has_realsense = config.settings['realsense']
realsense_url = config.settings['realsense_url']

print('Robot:', is_robot)
print('Lidar:',  has_lidar)
print('Arduino:',  has_arduino_at_115200)
print('Realsense:',  has_realsense)
print('Realsense URL:',  realsense_url)
    
if is_robot:
    from adafruit_motor import servo
    from adafruit_pca9685 import PCA9685
    from board import SCL, SDA
    import busio
    
if has_lidar:
    from adafruit_rplidar import RPLidar

    from memory import Memory
    mem = Memory()


    try:
        print("Loading Lidar libs")
        #lidar = RPLidar('/dev/ttyUSB0')
        lidar = RPLidar(None, '/dev/ttyUSB0', timeout=3)#, logging=True) 
        lidar.clear_input()
        print(lidar.health)
        print(lidar.info)
    except:
        print(traceback.format_exc())
        print("Failed.")




    def background_lidar_thread():

        #we'll set up a datastructure to track angle/distances
        #we'll floor all the angles to integers
        #also, normalise distances from 0 to 1
        #start at 1, get closer, as data comes in
        distances = [1]*360
        max_distance = 2000

        iterator = lidar.iter_measurements()

        counter = 0
    
        right_segment_distance = 1
        right_min = 15
        right_max = 45

        front_segment_distance = 1
        front_min = 345
        front_max = 15

        left_segment_distance = 1
        left_min = 315
        left_max = 345
    

        for new_scan, quality, angle, distance in iterator:

            #if distance > max_distance:
            #    max_distance = distance
            #    socketio.emit('lidar_data', {'data': distances})

            if quality > 0 and distance > 0:
                angle = int(angle)
                ratio = distance / max_distance
                distances[angle] = ratio if ratio <= 1 else 1

                if angle > left_min and angle <= left_max and ratio < left_segment_distance:
                    left_segment_distance = ratio

                if (angle > front_min or angle <= front_max) and ratio < front_segment_distance:
                    front_segment_distance = ratio

                if angle > right_min and angle <= right_max and ratio < right_segment_distance:
                    right_segment_distance = ratio

                counter = counter + 1

            if (counter == 360):
                mem.storeState("L", left_segment_distance)
                mem.storeState("F", front_segment_distance)
                mem.storeState("R", right_segment_distance)

                print(" L ", left_segment_distance, " F ", front_segment_distance, " R ", right_segment_distance)
                socketio.emit('lidar_data', {'data': distances})
                left_segment_distance = 1
                front_segment_distance = 1
                right_segment_distance = 1
                counter = 0


   
        
    
if has_arduino_at_115200:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    
    
def poll_arduino():
    try:
       ser.write(str.encode('1'))
    
       time.sleep(0.1)
    
       read_serial = ser.readline()
    
       read_json = json.loads(read_serial.decode())
       print (read_json)
       return read_json
    except Exception as e:
        print(e)
    
     
if has_arduino_at_115200:
    poll_arduino()
    
    
    
if is_robot:
    i2c = busio.I2C(SCL, SDA)
    
    pca = PCA9685(i2c)
    pca.frequency = 50
    
    servo0 = servo.Servo(pca.channels[0])
    servo1 = servo.Servo(pca.channels[1])
    servo2 = servo.Servo(pca.channels[2])
    servo3 = servo.Servo(pca.channels[3])
    servo4 = servo.Servo(pca.channels[4])
    servo5 = servo.Servo(pca.channels[5])
    servo6 = servo.Servo(pca.channels[6])
    servo7 = servo.Servo(pca.channels[7])
        
    servo0.angle = 90
    servo1.angle = 90
    servo2.angle = 90
    servo3.angle = 90
    servo4.angle = 90
    servo5.angle = 90
    servo6.angle = 90
    servo7.angle = 90





def poll_realsense():
    response = requests.post(url=realsense_url)
    print(response)
    return response






four_servo_mode = True
    
####################

RANGE=15
NUM_TIMES = 5
DELAY = 0.005
SHIFT_VAL = 0
CAMERA = 1



#####################
def close_running_threads():
    if is_robot:
        pca.deinit()
    if has_lidar:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        #client.close()

#Register the function to be called on exit
atexit.register(close_running_threads)


#####################


@socketio.event
def connect():
    global thread
    with thread_lock:
        if thread is None:
            thread = socketio.start_background_task(background_lidar_thread)
    emit('server_info', {'data': 'Connected!'})


@socketio.event
def disconnect_request():
    @copy_current_request_context
    def can_disconnect():
        disconnect()

    emit('server_info',
         {'data': 'Disconnected!'},
         callback=can_disconnect)




@app.route('/')
def index():
    print("Index()")
    print(session)

    if 'CAMERA' not in session:
        session['CAMERA'] = 1

    if 'MOTION' not in session:
        session['MOTION'] = 'walk'

    if 'MOTION2' not in session:
        session['MOTION2'] = 'walk'

    print ("INDEX MOTION: " + session['MOTION'])
    print ("INDEX MOTION2: " + session['MOTION2'])

    if 'MOTION' in session:
        load_id = session['MOTION']
        print ('MOTION', load_id)
        print('./motions/' + load_id)
        
        checkpoints = glob('./motions/' + load_id + '/*')
        print(checkpoints)
    
        cleanedList = [x for x in checkpoints if os.path.isdir(x)]
        sorted_saves = sorted(cleanedList, key=lambda x: x[1], reverse=True) 
    
        print(sorted_saves)
        try: 
            front_right = np.load(sorted_saves[0] + '/angle_front_right_180.npy', allow_pickle=True)
            front_left = np.load(sorted_saves[0] + '/angle_front_left_180.npy', allow_pickle=True)
            back_right = np.load(sorted_saves[0] + '/angle_back_right_180.npy', allow_pickle=True)
            back_left = np.load(sorted_saves[0] + '/angle_back_left_180.npy', allow_pickle=True)
        except:
            print("ERROR LOADING MOTION 1") 

    if 'MOTION2' in session:
        load_id2 = session['MOTION2']
        print ('MOTION2', load_id2)
        print('./motions/' + load_id2)
        
        checkpoints = glob('./motions/' + load_id2 + '/*')
        print(checkpoints)
    
        cleanedList = [x for x in checkpoints if os.path.isdir(x)]
        sorted_saves = sorted(cleanedList, key=lambda x: x[1], reverse=True) 
    
        print(sorted_saves)

        try:
            gripper_1 = np.load(sorted_saves[0] + '/angle_front_right_180.npy', allow_pickle=True)
            gripper_2 = np.load(sorted_saves[0] + '/angle_front_left_180.npy', allow_pickle=True)
            gripper_3 = np.load(sorted_saves[0] + '/angle_back_right_180.npy', allow_pickle=True)
            gripper_4 = np.load(sorted_saves[0] + '/angle_back_left_180.npy', allow_pickle=True)
        except:
            print("ERROR LOADING MOTION 1") 
       
    try: 
        with open("defaults/defaults.txt") as f:
            print("OPENED DEFAULTS FILE")
            forward = f.readline().rstrip("\n") 
            backwards = f.readline().rstrip("\n") 
            left = f.readline().rstrip("\n") 
            right = f.readline().rstrip("\n") 
        
            session['FORWARD'] = forward
            session['BACKWARDS'] = backwards
            session['LEFT'] = left
            session['RIGHT'] = right
    
    except:
        print("ERROR:Missing defaults file?")

    #set some defaulrs
    try:
        four_servo_mode = session['MODE'] is None or session['MODE'] == "Four" 
        if session['MODE'] is None:
            session['MODE'] = "Four"
    except:
        session['MODE'] = "Four"
        four_servo_mode = True

    print ("Mode:", four_servo_mode)
    
    if four_servo_mode:
        gripper_1 = front_right
        gripper_2 = front_left
        gripper_3 = back_right
        gripper_4 = back_left 

    if 'NUM_TIMES' not in session:
        session['NUM_TIMES'] = NUM_TIMES
    
    if 'DELAY' not in session:
        session['DELAY'] = DELAY

    if 'RANGE' not in session:
        session['RANGE'] = RANGE
    
    if 'SHIFT_VAL' not in session:
        session['SHIFT_VAL'] = SHIFT_VAL

    if 'MOTION' not in session:
        session['MOTION'] = 'walk'

    if 'MOTION2' not in session:
        session['MOTION2'] = 'walk'

    if 'COMBONAME' not in session:
        session['COMBONAME'] = 'walkwalk'

    if 'front_right_adjuster' not in session:
        session['front_right_adjuster'] = 0
    
    if 'front_left_adjuster' not in session:
        session['front_left_adjuster'] = 0

    if 'back_right_adjuster' not in session:
        session['back_right_adjuster'] = 0

    if 'back_left_adjuster' not in session:
        session['back_left_adjuster'] = 0


    print ("INDEX MOTION: " + session['MOTION'])
    print ("INDEX MOTION2: " + session['MOTION2'])

    print ("COMBONAME: " + session['COMBONAME'])

    url4 = 'index.html'
    url8 = 'index8.html'
    print (url4 if four_servo_mode else url8)


    data= {"MOTION"     :session['MOTION'],
           "MOTION2"    :session['MOTION2'],
           "COMBONAME"  :session['COMBONAME'],
           "MODE"       :session['MODE'],
           "RANGE"      :session['RANGE'],
           "TIMES_VAL"  :session['NUM_TIMES'],
           "SHIFT_VAL"  :session['SHIFT_VAL'],
           "DELAY_VAL"  :session['DELAY'],
           "CAMERA"     :session['CAMERA'],
           "FORWARD"    :session['FORWARD'],
           "BACKWARDS"  :session['BACKWARDS'],
           "LEFT"       :session['LEFT'],
           "RIGHT"      :session['RIGHT'],
           "fr_adjust"  :session['front_right_adjuster'],
           "fl_adjust"  :session['front_left_adjuster'],
           "br_adjust"  :session['back_right_adjuster'],
           "bl_adjust"  :session['back_left_adjuster'],
           "x"          :json.dumps(np.arange(front_right.size).tolist()),
           "front_right":json.dumps(front_right.tolist()),
           "front_left" :json.dumps(front_left.tolist()),
           "back_right" :json.dumps(back_right.tolist()),
           "back_left"  :json.dumps(back_left.tolist()),
           "gripper_1"  :json.dumps(gripper_1.tolist()),
           "gripper_2"  :json.dumps(gripper_2.tolist()),
           "gripper_3"  :json.dumps(gripper_3.tolist()),
           "gripper_4"  :json.dumps(gripper_4.tolist())}

    print (data)

    return render_template(url4 if four_servo_mode else url8, leg_data=data)








@app.route('/settoadjusters', methods=['POST', 'GET'])
def settoadjusters():
    if is_robot:
        servo0.angle = 90 + session['front_right_adjuster']
        servo1.angle = 90 - session['front_left_adjuster']
        servo2.angle = 90 + session['back_right_adjuster']
        servo3.angle = 90 - session['back_left_adjuster']
        servo4.angle = 90
        servo5.angle = 90
        servo6.angle = 90
        servo7.angle = 90

    results = {'processed': 'true'}
    return jsonify(results)


@app.route('/stop', methods=['POST', 'GET'])
def stop():

    stop_robot_event.set()

    if is_robot:
        servo0.angle = 90
        servo1.angle = 90
        servo2.angle = 90
        servo3.angle = 90
        servo4.angle = 90
        servo5.angle = 90
        servo6.angle = 90
        servo7.angle = 90

    results = {'processed': 'true'}
    return jsonify(results)

@app.route('/setrepeat', methods=['POST', 'GET'])
def setrepeat():
    data = request.get_json()
    print (data) 
    session['NUM_TIMES'] = data[0]['times'] 
    print(session['NUM_TIMES'])
    results = {'processed': 'true'}
    return jsonify(results)


@app.route('/setdelay', methods=['POST', 'GET'])
def setdelay():
    data = request.get_json()
    print (data)
    session['DELAY'] = data[0]['delay']
    print(session['DELAY'])
    results = {'processed': 'true'}
    return jsonify(results)


@app.route('/updateForward', methods=['POST'])
def updateForward():
    data = request.get_json()
    print (data)
    session['FORWARD'] = data[0]['data']
    print(session['FORWARD'])
    try: 
       
        defaultsArray = []

        with open("defaults/defaults.txt") as f:
            print("OPENED DEFAULTS FILE")
            forward = f.readline().rstrip("\n") 
            backwards = f.readline().rstrip("\n") 
            left = f.readline().rstrip("\n") 
            right = f.readline().rstrip("\n") 
        
            defaultsArray = [session['FORWARD'], backwards, left, right]
    
        npArray = np.array(defaultsArray)

        np.savetxt("defaults/defaults.txt", npArray, fmt="%s")

    except:
        print("Failed to update Forward")

    results = {'processed': 'true'}
    return jsonify(results)

@app.route('/updateBackwards', methods=['POST'])
def updateBackwards():
    data = request.get_json()
    print (data)
    session['BACKWARDS'] = data[0]['data']
    print(session['BACKWARDS'])
    try: 
       
        defaultsArray = []

        with open("defaults/defaults.txt") as f:
            print("OPENED DEFAULTS FILE")
            forward = f.readline().rstrip("\n") 
            backwards = f.readline().rstrip("\n") 
            left = f.readline().rstrip("\n") 
            right = f.readline().rstrip("\n") 
        
            defaultsArray = [forward, session['BACKWARDS'], left, right]
    
        npArray = np.array(defaultsArray)

        np.savetxt("defaults/defaults.txt", npArray, fmt="%s")

    except:
        print("Failed to update Backwards")
    
    results = {'processed': 'true'}
    return jsonify(results)

@app.route('/updateLeft', methods=['POST'])
def updateLeft():
    data = request.get_json()
    print (data)
    session['LEFT'] = data[0]['data']
    print(session['LEFT'])
    try: 
       
        defaultsArray = []

        with open("defaults/defaults.txt") as f:
            print("OPENED DEFAULTS FILE")
            forward = f.readline().rstrip("\n") 
            backwards = f.readline().rstrip("\n") 
            left = f.readline().rstrip("\n") 
            right = f.readline().rstrip("\n") 
        
            defaultsArray = [forward, backwards, session['LEFT'], right]
    
        npArray = np.array(defaultsArray)

        np.savetxt("defaults/defaults.txt", npArray, fmt="%s")

    except:
        print("Failed to update Left")
    
    results = {'processed': 'true'}
    return jsonify(results)

@app.route('/updateRight', methods=['POST'])
def updateRight():
    data = request.get_json()
    print (data)
    session['RIGHT'] = data[0]['data']
    print(session['RIGHT'])
    try: 
       
        defaultsArray = []

        with open("defaults/defaults.txt") as f:
            print("OPENED DEFAULTS FILE")
            forward = f.readline().rstrip("\n") 
            backwards = f.readline().rstrip("\n") 
            left = f.readline().rstrip("\n") 
            right = f.readline().rstrip("\n") 
        
            defaultsArray = [forward, backwards, left, session['RIGHT']]
    
        npArray = np.array(defaultsArray)

        np.savetxt("defaults/defaults.txt", npArray, fmt="%s")

    except:
        print("Failed to update Right")
    
    results = {'processed': 'true'}
    return jsonify(results)




@app.route('/setfr', methods=['POST', 'GET'])
def setfr():
    data = request.get_json()
    print (data)
    session['front_right_adjuster'] = int(data[0]['fr'])
    print(session['front_right_adjuster'])
    results = {'processed': 'true'}
    return jsonify(results)

@app.route('/setfl', methods=['POST', 'GET'])
def setfl():
    data = request.get_json()
    print (data)
    session['front_left_adjuster'] = int(data[0]['fl'])
    print(session['front_left_adjuster'])
    results = {'processed': 'true'}
    return jsonify(results)

@app.route('/setbr', methods=['POST', 'GET'])
def setbr():
    data = request.get_json()
    print (data)
    session['back_right_adjuster'] = int(data[0]['br'])
    print(session['back_right_adjuster'])
    results = {'processed': 'true'}
    return jsonify(results)

@app.route('/setbl', methods=['POST', 'GET'])
def setbl():
    data = request.get_json()
    print (data)
    session['back_left_adjuster'] = int(data[0]['bl'])
    print(session['back_left_adjuster'])
    results = {'processed': 'true'}
    return jsonify(results)


@app.route('/toggleMode', methods=['POST'])
def toggle_mode():
    
    #request.form for POST, not request.args!
    print (request.form)
    session['MODE'] = request.form.get('servo_mode')#data[0]['servo_mode']
    print("SETTING MODE TO ", session['MODE'])
    return index()


@app.route('/toggleCamera', methods=['GET'])
def toggle_camera():
    print('TOGGLING CAMERA') 
    session['CAMERA'] = 0 if session['CAMERA'] == 1 else 1 
    print("SETTING CAMERA MODE TO ", session['CAMERA'])
    results = {'processed': 'true'}
    return jsonify(results)


@app.route('/selectGripper', methods=['POST'])
def selectGripper():
    
    #request.form for POST, not request.args!
    print (request.form)
    session['MOTION2'] = request.form.get('gripperId')#data[0]['servo_mode']
    print("SETTING MOTION2 TO ", session['MOTION2'])
    return index()



@app.route('/combo', methods=['POST'])
def combo():
    
    print(session)
    #request.form for POST, not request.args!
    print (request.form)
    session['COMBONAME'] = request.form.get('comboId')#data[0]['servo_mode']
    print("SETTING COMBO_NAME TO ", session['COMBONAME'])
    load_id = session['COMBONAME']
        
    checkpoints = glob('./motions8/' + load_id + '/*')
    print(checkpoints)
    
    cleanedList = [x for x in checkpoints if os.path.isdir(x)]
    sorted_saves = sorted(cleanedList, key=lambda x: x[1], reverse=True) 
    
    print(sorted_saves)
    comboArray = ['walk', 'walk']
    try: 

        print("LOADING ", sorted_saves[0] + "/combo.txt")
        with open(sorted_saves[0] + "/combo.txt") as f:
            print("OPENED")
            motion1 = f.readline().rstrip("\n") 
            motion2 = f.readline().rstrip("\n") 
            print( motion1 )
            print( motion2 )
    
            comboArray[0] = motion1
            comboArray[1] = motion2
    except:
        print("FAIL combo load")

    #load text file, get motions 1 and 2
    session['MOTION'] = comboArray[0]
    session['MOTION2'] = comboArray[1]

    return index()







@app.route('/getdirs', methods=['POST', 'GET'])
def getdirs():
    #data = request.get_json()
    #print (data)
    listing = get_directory_listing("./motions")
    print (jsonify(listing))
    return jsonify(listing)

@app.route('/getcombodirs', methods=['POST', 'GET'])
def getcombodirs():
    listing = get_directory_listing("./motions8")
    print (jsonify(listing))
    return jsonify(listing)


@app.route('/load', methods=['POST', 'GET'])
def load():
    print(request.form)
    load_id = request.form['id']
    
    session['MOTION'] = load_id
    print (load_id)

    return index()


@app.route('/save8', methods=['POST', 'GET'])
def save8():
    print("SAVE8")
    print(request.form)
    print(session)
    
    save_motion_id = request.form['MOTION1']
    save_motion2_id = request.form['MOTION2']
    save_motion_combo_name = request.form['COMBONAME']
   
    if save_motion_id == None or 'object' in save_motion_id:
        save_motion_id = 'walk' 
    if save_motion2_id == None or 'object' in save_motion2_id:
        save_motion2_id = 'walk' 

    if save_motion_combo_name == None:
        save_motion_combo_name = 'walkwalk' 

    comboArray = [save_motion_id, save_motion2_id]

    npArray = np.array(comboArray)
    date = datetime.now().strftime("%Y%m%d%I%M%S%f")

    #dirs must exist first
    dirspath = './motions8/' + save_motion_combo_name + '/' + date + '/' 
    os.makedirs(dirspath)

    np.savetxt(dirspath + "combo.txt", npArray, fmt="%s")

#    file = open(dirspath + "combo.txt", "w+")
#    content = str(npArray)
#    file.write(content)
#    file.close()


    return index()


@app.route('/save', methods=['POST', 'GET'])
def save():
    print(request.form)

    front_right = request.form['front_right']
    front_left = request.form['front_left']
    back_right = request.form['back_right']
    back_left = request.form['back_left']
   
    
    
    try:
        print(session)
        print(session['MODE'] is None)
        print(session['MODE'] == "Four")
        four_servo_mode = session['MODE'] is None or session['MODE'] == "Four" 
    except:
        session['MODE'] = "Four"
        four_servo_mode = True

    print ("Mode:", four_servo_mode)
   

    if four_servo_mode:
        gripper_1 = front_right
        gripper_2 = front_left
        gripper_3 = back_right
        gripper_4 = back_left 
    else:
        try:
            gripper_1 = request.form['gripper_1']
            gripper_2 = request.form['gripper_2']
            gripper_3 = request.form['gripper_3']
            gripper_4 = request.form['gripper_4']
        except:
            gripper_1 = front_right
            gripper_2 = front_left
            gripper_3 = back_right
            gripper_4 = back_left 


    #print(front_right)

    save_id = request.form['id']
    print (save_id)

    session['MOTION'] = save_id

    front_right_list = json.loads('[' + front_right + ']')
    front_left_list = json.loads('[' + front_left + ']')
    back_right_list = json.loads('[' + back_right + ']')
    back_left_list = json.loads('[' + back_left + ']')
    gripper_1_list = json.loads('[' + gripper_1 + ']')
    gripper_2_list = json.loads('[' + gripper_2 + ']')
    gripper_3_list = json.loads('[' + gripper_3 + ']')
    gripper_4_list = json.loads('[' + gripper_4 + ']')

    #print(front_right_list)

    print('./motions/' + save_id)

    date = datetime.now().strftime("%Y%m%d%I%M%S%f")

    #dirs must exist first
    dirspath = './motions/' + save_id + '/' + date + '/' 
    os.makedirs(dirspath)

    front_right_list = [int(x) for x in front_right_list]
    front_left_list = [int(x) for x in front_left_list]
    back_right_list = [int(x) for x in back_right_list]
    back_left_list = [int(x) for x in back_left_list]

    np.save(dirspath + 'angle_front_right_180.npy', front_right_list) 
    np.save(dirspath + 'angle_front_left_180.npy', front_left_list) 
    np.save(dirspath + 'angle_back_right_180.npy', back_right_list) 
    np.save(dirspath + 'angle_back_left_180.npy', back_left_list) 


    np.savetxt(dirspath + 'angle_front_right_180.txt', front_right_list)
    np.savetxt(dirspath + 'angle_front_left_180.txt', front_left_list)
    np.savetxt(dirspath + 'angle_back_right_180.txt', back_right_list)
    np.savetxt(dirspath + 'angle_back_left_180.txt', back_left_list)




    return index()


@app.route('/fidget', methods=['POST', 'GET'])
def fidget():
#    data = request.get_json()
#    front_right_update = data[0]
#    front_left_update = data[1]
#    back_right_update = data[2]
#    back_left_update = data[3]
#
#    front_right = front_right_update['front_right']
#    front_left = front_left_update['front_left']
#    back_right = back_right_update['back_right']
#    back_left = back_left_update['back_left']
#    
#    for i in range (6): 
#        #adjust
#        front_right = np.array(front_right)
#        front_left = np.array(front_left)
#        back_right = np.array(back_right)
#        back_left = np.array(back_left)
#    
#        #too volatile
#
#        #if random.randint(0,1) == 1:
#        #    front_right = 180 - front_right
#        #if random.randint(0,1) == 1:
#        #    front_left = 180 - front_left
#        #if random.randint(0,1) == 1:
#        #    back_right = 180 - back_right
#        #if random.randint(0,1) == 1:
#        #    back_left = 180 - back_left
#    
#    
#        if random.randint(0,1) == 1:
#            front_right = np.flip(front_right)
#        if random.randint(0,1) == 1:
#            front_left = np.flip(front_left)
#        if random.randint(0,1) == 1:
#            back_right = np.flip(back_right)
#        if random.randint(0,1) == 1:
#            back_left = np.flip(back_left)
#   
#
#        front_right = front_right + int(session['front_right_adjuster'])
#        front_left = front_left + int(session['front_left_adjuster'])
#        back_right = back_right + int(session['back_right_adjuster'])
#        back_left = back_left + int(session['back_left_adjuster'])
#
#        front_left = 180-front_left
#        back_left = 180-back_left
#
#        results = {'processed': 'true'}
#       
#        NUM_TIMES = int(session['NUM_TIMES'])
#        print(NUM_TIMES)
#        r_front_right = np.tile(front_right, NUM_TIMES)
#        r_front_left = np.tile(front_left, NUM_TIMES)
#        r_back_right = np.tile(back_right, NUM_TIMES)
#        r_back_left = np.tile(back_left, NUM_TIMES)
#    
#    
#        for i in range(len(r_front_right)):
#            servo0.angle = int(r_front_right[i])
#            servo1.angle = int(r_front_left[i])
#            servo2.angle = int(r_back_right[i])
#            servo3.angle = int(r_back_left[i])
#            
#            DELAY = float(session['DELAY'])
#            time.sleep(DELAY)
#        
    results = {'processed': 'false'}
    return jsonify(results)



@app.route('/runBrain', methods=['POST'])
def runBrain():

    @copy_current_request_context
    def runBrainTask():

        @copy_current_request_context
        def runMotionFromCategory(category):
            #for random selection by category
    
            #path = './motions/'
            #children = []
    
            #for name in os.listdir(path):
            #    subpath = os.path.join(path, name)
            #    if os.path.isdir(subpath) and category in subpath.upper():
            #        children.append(subpath)
    
            #print(category, ':', children)
    
            #child = random.choice(children)
    
            #if child is None:
            #    print("No Motions available for ", category)
    
            #print ('Running ', child)
            #
            #checkpoints = glob(child+'/*')
            #print(checkpoints)
        
            #cleanedList = [x for x in checkpoints if os.path.isdir(x)]
            #sorted_saves = sorted(cleanedList, key=lambda x: x[1], reverse=True) 
        
            #print(sorted_saves)
            try: 
                #front_right = np.load(sorted_saves[0] + '/angle_front_right_180.npy', allow_pickle=True)
                #front_left = np.load(sorted_saves[0] + '/angle_front_left_180.npy', allow_pickle=True)
                #back_right = np.load(sorted_saves[0] + '/angle_back_right_180.npy', allow_pickle=True)
                #back_left = np.load(sorted_saves[0] + '/angle_back_left_180.npy', allow_pickle=True)

                category_dir = "./motions/" + session[category] 
                checkpoints = glob(category_dir + '/*')
                checkpoint_dirs = [x for x in checkpoints if os.path.isdir(x)]
                sorted_saves = sorted(checkpoint_dirs, key=lambda x: x[1], reverse=True) 



                front_right = np.load(sorted_saves[0] + '/angle_front_right_180.npy', allow_pickle=True)
                front_left = np.load(sorted_saves[0] + '/angle_front_left_180.npy', allow_pickle=True)
                back_right = np.load(sorted_saves[0] + '/angle_back_right_180.npy', allow_pickle=True)
                back_left = np.load(sorted_saves[0] + '/angle_back_left_180.npy', allow_pickle=True)
        
                #adjust   add all, since subtracting lefts from 180 later.
                front_right = np.array(front_right) + int(session['front_right_adjuster'])
                front_left = np.array(front_left) + int(session['front_left_adjuster'])
                back_right = np.array(back_right) + int(session['back_right_adjuster'])
                back_left = np.array(back_left) + int(session['back_left_adjuster'])

                front_left = 180-front_left
                back_left = 180-back_left

                if is_robot:
                    for i in range(len(front_right)):
                        if stop_robot_event.is_set(): 
                            print(f'end {threading.current_thread().name} ')
                            return
   
                        servo0.angle = int(front_right[i])
                        servo1.angle = int(front_left[i])
                        servo2.angle = int(back_right[i])
                        servo3.angle = int(back_left[i])
                        
                        DELAY = float(session['DELAY'])
                        time.sleep(DELAY)

            except:
                print(traceback.format_exc())
                print("ERROR LOADING ", child) 
    
    
    
    

        if (has_lidar):
            while True:

                if stop_robot_event.is_set():
                    print(f'end {threading.current_thread().name} ')
                    return


                try:
                    TOO_CLOSE = 0.2
                    priority = 0

                    L = float(mem.retrieveState('L'))
                    F = float(mem.retrieveState('F'))
                    R = float(mem.retrieveState('R'))

                    print (L, " ", F, " ", R)


                    # High Priority '0'
                    if F < TOO_CLOSE:
                        runMotionFromCategory("BACKWARDS")
                        priority = 1
                    if L < TOO_CLOSE:
                        #runMotionFromCategory("BACK")
                        runMotionFromCategory("RIGHT")
                        priority = 1
                    if R < TOO_CLOSE:
                        #runMotionFromCategory("BACK")
                        runMotionFromCategory("LEFT")
                        priority = 1
                    
                    # Medium Priority '1'  (priority is still 0)
                    if priority == 0 and L < R and L < F:
                        runMotionFromCategory("RIGHT")
                        priority = 2
                     
                    if priority == 0 and R < L and R < F:
                        runMotionFromCategory("LEFT")
                        priority = 2
                    
                    # Low Priority '2'  (priority is still 0)
                    if priority == 0 and L < F and R < F:
                        runMotionFromCategory("FORWARD")
                    
                    if priority == 0 and F > 100:
                        runMotionFromCategory("FORWARD")
                    elif priority == 0 and F < 50:
                        runMotionFromCategory("BACKWARDS")


                except:
                    print(traceback.format_exc())
                    print("ERROR RETRIEVING LIDAR STATE")
                    results = {'processed': 'true'}
                    return jsonify(results)


        elif (has_realsense):
            poll_realsense()
   
        elif (has_arduino_at_115200):
        
            TOO_CLOSE=40
            while True:
    
                if stop_robot_event.is_set():
                    print(f'end {threading.current_thread().name} ')
                    return
        
                priority = 0
               
                read_json = poll_arduino()
                try:
                    R = int(read_json['R'])
                    F = int(read_json['F'])
                    L = int(read_json['L'])
                except:
                    print("ERROR POLLING ARDUINO")
                    results = {'processed': 'true'}
                    return jsonify(results)
            
            
            
                # High Priority '0'
                if F < TOO_CLOSE:
                    runMotionFromCategory("BACKWARDS")
                    priority = 1
                if L < TOO_CLOSE:
                    #runMotionFromCategory("BACK")
                    runMotionFromCategory("RIGHT")
                    priority = 1
                if R < TOO_CLOSE:
                    #runMotionFromCategory("BACK")
                    runMotionFromCategory("LEFT")
                    priority = 1
                
                # Medium Priority '1'  (priority is still 0)
                if priority == 0 and L < R and L < F:
                    runMotionFromCategory("RIGHT")
                    priority = 2
                 
                if priority == 0 and R < L and R < F:
                    runMotionFromCategory("LEFT")
                    priority = 2
                
                # Low Priority '2'  (priority is still 0)
                if priority == 0 and L < F and R < F:
                    runMotionFromCategory("FORWARD")
                
                if priority == 0 and F > 100:
                    runMotionFromCategory("FORWARD")
                elif priority == 0 and F < 50:
                    runMotionFromCategory("BACKWARDS")


    stop_robot_event.clear()
    t = Thread(target=runBrainTask, args=(), daemon=True)  #check the syntax
    t.start()

    results = {'processed': 'true'}
    return jsonify(results)





@app.route('/runadjustedmotion', methods=['POST', 'GET'])
def runadjustedmotion():
    data = request.get_json()
    return runadjustedmotiondirect(data)

def runadjustedmotiondirect(data):
    @copy_current_request_context
    def runMotionTask(data):
    
        front_right_update = data[0]
        front_left_update = data[1]
        back_right_update = data[2]
        back_left_update = data[3]
        
        
        try:
            print(session)
            four_servo_mode = session['MODE'] is None or session['MODE'] == "Four" 
        except:
            session['MODE'] = "Four"
            four_servo_mode = True
    
        print ("Mode:", four_servo_mode)
        
        try:
            gripper_1_update = data[4]
            gripper_2_update = data[5]
            gripper_3_update = data[6]
            gripper_4_update = data[7]
        except:
            print("NO GRIPPER DATA")
            gripper_1_update = front_right_update
            gripper_2_update = front_left_update
            gripper_3_update = back_right_update
            gripper_4_update = back_left_update 
    
        front_right = front_right_update['front_right']
        front_left = front_left_update['front_left']
        back_right = back_right_update['back_right']
        back_left = back_left_update['back_left']
    
        #adjust   - add them cause we're subtracting from 180 later.
        front_right = np.array(front_right) + int(session['front_right_adjuster'])
        front_left = np.array(front_left) + int(session['front_left_adjuster'])
        back_right = np.array(back_right) + int(session['back_right_adjuster'])
        back_left = np.array(back_left) + int(session['back_left_adjuster'])
    
    
        if not four_servo_mode:
            gripper_1 = gripper_1_update['gripper_1']
            gripper_2 = gripper_2_update['gripper_2']
            gripper_3 = gripper_3_update['gripper_3']
            gripper_4 = gripper_4_update['gripper_4']
    
    
       
        NUM_TIMES = int(session['NUM_TIMES'])
        print(NUM_TIMES)
        r_front_right = np.tile(front_right, NUM_TIMES)
        r_front_left = np.tile(front_left, NUM_TIMES)
        r_back_right = np.tile(back_right, NUM_TIMES)
        r_back_left = np.tile(back_left, NUM_TIMES)
   
        #maybe make these configurable, cause you could set up the servo backwards
        r_front_left = 180-r_front_left
        r_back_left = 180-r_back_left
    
    
        if not four_servo_mode:
            gripper_1 = np.tile(gripper_1, NUM_TIMES)
            gripper_2 = np.tile(gripper_2, NUM_TIMES)
            gripper_3 = np.tile(gripper_3, NUM_TIMES)
            gripper_4 = np.tile(gripper_4, NUM_TIMES)
    
        if is_robot: 
            for i in range(len(r_front_right)):
                if stop_robot_event.is_set(): 
                    print(f'end {threading.current_thread().name} ')
                    return
   
                #print(i)
                servo0.angle = int(r_front_right[i])
                servo1.angle = int(r_front_left[i])
                servo2.angle = int(r_back_right[i])
                servo3.angle = int(r_back_left[i])
                if not four_servo_mode:
                    servo4.angle = int(gripper_1[i])
                    servo5.angle = int(gripper_2[i])
                    servo6.angle = int(gripper_3[i])
                    servo7.angle = int(gripper_4[i])
                
                DELAY = float(session['DELAY'])
                time.sleep(DELAY)

    stop_robot_event.clear()
    t = Thread(target=runMotionTask, args=(data,), daemon=True)  #check the syntax
    t.start()
    
    results = {'processed': 'true'}

    return jsonify(results)






def get_directory_listing(path):
    children = []

    for name in os.listdir(path):
        subpath = os.path.join(path, name)
        if os.path.isdir(subpath):
            children.append({
                'id': name, 
                'text': name,
                'parent' : '#',
                'children': [] #get_directory_listing(subpath)
            })

    return children



lastfile = "static/1.jpg"

def save_image():
    timestr = time.strftime("%Y%m%d-%H%M%S")
    lastfile = "static/snap_" + timestr + ".jpg"
    camera = picamera.PiCamera()
    camera.resolution = (640, 480)
    camera.start_preview()
    sleep(2)
    camera.capture(lastfile)
    camera.stop_preview()
    camera.close()

    return(lastfile)


@app.route('/snapshot')
def snapshot():
    return send_file(save_image())

@app.route("/snapshot2")
def getImage():
     camera.capture(lastfile)
     camera.close()
     return send_file(lastfile)



if __name__ == '__main__':
    #executor = Executor(app)
    socketio.run(app, debug=True, host='0.0.0.0')
    #app.run(debug=True, host='0.0.0.0')
