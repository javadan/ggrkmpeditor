from flask import Flask, render_template, request, jsonify, session, url_for, send_file, copy_current_request_context
from flask_session.__init__ import Session
from flask_socketio.__init__ import SocketIO, emit, join_room, leave_room, close_room, rooms, disconnect
from flask_executor import Executor

import eventlet
from eventlet import wsgi
from eventlet import websocket
eventlet.monkey_patch()
from threading import Thread, Event, Lock, Condition
import threading
_threading = eventlet.patcher.original('threading')

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
import logging

    
is_pca9685_robot = config.settings['pca9685_robot']
is_scservo_robot = config.settings['scservo_robot'] # < for Feetech Smart bus servos (SMT, RS485)
is_gpio_robot = config.settings['gpio_on_5_6_13_19_robot'] # corresponding to servos 1,2,3,4
has_lidar = config.settings['lidar']
has_arduino_at_115200 = config.settings['arduino_115200']
has_realsense = config.settings['realsense']
realsense_url = config.settings['realsense_url']
has_picamera = config.settings['picamera']
has_libcamera = config.settings['libcamera']
#is_streaming_libcamera = config.settings['streaming']
has_ultrasound_echo_on_pin_22 = config.settings['ultrasound_echo_on_pin_22']
has_ultrasound_trigger_on_pin_27 = config.settings['ultrasound_trigger_on_pin_27']
has_continuous_servo_on_pin_25 = config.settings['continuous_servo_on_pin_25']
has_switches_on_pins_23_34 = config.settings['has_switches_on_pins_23_34']




print('PCA9685 Robot:', is_pca9685_robot)
print('SCServo Robot:',  is_scservo_robot)
print('GPIO Robot:',  is_gpio_robot)
print('Lidar:',  has_lidar)
print('Arduino:',  has_arduino_at_115200)
print('Realsense:',  has_realsense)
print('Realsense URL:',  realsense_url)
print('Has picamera (Legacy RPi 32 bit):',  has_picamera)
print('Has libcamera (RPi 64 bit):',  has_libcamera)
#print('(TODO)Is streaming libcamera MJPEG (RPi 64 bit):',  is_streaming_libcamera)
print('Ultrasound sensor echo on pin 22:',  has_ultrasound_echo_on_pin_22)
print('Ultrasound sensor trigger on pin 27:',  has_ultrasound_trigger_on_pin_27)
#gripper
print('Has continuous servo on pin 25:',  has_continuous_servo_on_pin_25)
print('Has switches on pin 23/24:',  has_switches_on_pins_23_34)

if has_ultrasound_echo_on_pin_22 and has_ultrasound_trigger_on_pin_27:
    import board
    import RPi.GPIO as GPIO           

    from memory import Memory
    mem = Memory()

    def distance():
        GPIO_TRIGGER = 27
        GPIO_ECHO = 22

        GPIO.output(GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)
     
        StartTime = time.time()
        StopTime = time.time()
     
        while GPIO.input(GPIO_ECHO) == 0:
            StartTime = time.time()
     
        while GPIO.input(GPIO_ECHO) == 1:
            StopTime = time.time()
     
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
     
        return distance

    def background_ultrasound_thread():
        GPIO_TRIGGER = 27
        GPIO_ECHO = 22

        GPIO.setmode(GPIO.BCM)     
        GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(GPIO_ECHO, GPIO.IN) 
        while True:
            dist = distance()
            mem.storeState("F", dist)
            time.sleep(2)



if has_continuous_servo_on_pin_25:
    import board
    import pwmio
    from adafruit_motor import servo
    import RPi.GPIO as GPIO           
    
    pwm = pwmio.PWMOut(board.D25, frequency=50)
    my_servo = servo.ContinuousServo(pwm, min_pulse = 500, max_pulse = 2500)

    GPIO.setmode(GPIO.BCM)     
    GPIO.setup(23, GPIO.IN)  
    GPIO.setup(24, GPIO.IN)  
    
 
     #if GPIO.input(23) or GPIO.input(24): 

#    my_servo.throttle = 1.0
#    time.sleep(2.0)
 



if has_picamera:
    import picamera

if has_libcamera:
    print("Importing picamera2")
    from picamera2 import Picamera2, Preview
   
    def capture(camera, path='static/'):
        timestr = time.strftime("%Y%m%d-%H%M%S")
        camera.start()
        timestamp = datetime.now().isoformat(timespec='seconds')
        print('%s Image captured' % timestamp)
    
        file_path = os.path.join(path, '%s.jpg' % timestamp)
        metadata = camera.capture_file(file_path)
        print(metadata)
        camera.stop()

        return file_path
    




if is_gpio_robot:
    from adafruit_motor import servo
    import board
    import pigpio
   
    servo0 = 5
    servo1 = 6
    servo2 = 13
    servo3 = 19

    pwm0 = pigpio.pi()
    pwm0.set_mode(servo0, pigpio.OUTPUT)
    pwm1 = pigpio.pi()
    pwm1.set_mode(servo1, pigpio.OUTPUT)
    pwm2 = pigpio.pi()
    pwm2.set_mode(servo2, pigpio.OUTPUT)
    pwm3 = pigpio.pi()
    pwm3.set_mode(servo3, pigpio.OUTPUT)

    pwm0.set_PWM_frequency( servo0, 50 )
    pwm1.set_PWM_frequency( servo1, 50 )
    pwm2.set_PWM_frequency( servo2, 50 )
    pwm3.set_PWM_frequency( servo3, 50 )

    def remap(x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def remap_to_pwm(x):
        return remap(x, 0, 179, 1000, 2000);



if is_pca9685_robot:
    from adafruit_motor import servo
    from adafruit_pca9685 import PCA9685
    from board import SCL, SDA
    import busio

elif is_scservo_robot:
    import serial
    from scservo_sdk import *

    def pingServo(SCS_ID):
        # Get SCServo model number
        scs_model_number, scs_comm_result, scs_error = packetHandler.ping(portHandler, SCS_ID)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))
        else:
            print("[ID:%03d] ping Succeeded. SCServo model number : %d" % (SCS_ID, scs_model_number))
        
    def remap(x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
            
    #The SC Servos range up to 4096, with 2048 being in the middle 
    #For us, with 180 degree servos, 1024 might be the middle.  
    
    # Control table address
    ADDR_SCS_TORQUE_ENABLE     = 40
    ADDR_STS_GOAL_ACC          = 41
    ADDR_STS_GOAL_POSITION     = 42
    ADDR_STS_GOAL_SPEED        = 46
    ADDR_STS_PRESENT_POSITION  = 56
    
    # Default setting
    SCS1_ID                     = 1                 
    SCS2_ID                     = 2                 
    SCS3_ID                     = 3                 
    SCS4_ID                     = 4                 
    BAUDRATE                    = 115200           
    DEVICENAME                  = '/dev/ttyUSB0'    
                                                    
    
    SCS_MINIMUM_POSITION_VALUE  = 1536               
    SCS_MAXIMUM_POSITION_VALUE  = 2560
    
    SCS_MOVING_STATUS_THRESHOLD = 10                # SCServo moving status threshold
    SCS_MOVING_SPEED            = 0                 # SCServo moving speed
    SCS_MOVING_ACC              = 0                 # SCServo moving acc
    protocol_end                = 0                 # SCServo bit end(STS/SMS=0, SCS=1)
    
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(protocol_end)
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_STS_GOAL_POSITION, 2)
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_STS_PRESENT_POSITION, 4)
    
    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        quit()
    
    
    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        quit()


    # Add parameter storage for SCServo#1 present position value
    scs_addparam_result = groupSyncRead.addParam(SCS1_ID)
    if scs_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % SCS1_ID)
        quit()
    
    # Add parameter storage for SCServo#2 present position value
    scs_addparam_result = groupSyncRead.addParam(SCS2_ID)
    if scs_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % SCS2_ID)
        quit()

    # Add parameter storage for SCServo#3 present position value
    scs_addparam_result = groupSyncRead.addParam(SCS3_ID)
    if scs_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % SCS3_ID)
        quit()
    
    # Add parameter storage for SCServo#4 present position value
    scs_addparam_result = groupSyncRead.addParam(SCS4_ID)
    if scs_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % SCS4_ID)
        quit()

    
    # Get SCServo model number
    pingServo(SCS1_ID) 
    pingServo(SCS2_ID) 
    pingServo(SCS3_ID) 
    pingServo(SCS4_ID) 

    
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
        
     
    poll_arduino()
    
    



if is_pca9685_robot:
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

    

####################


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

ultrasound_thread = None
ultrasound_thread_lock = Lock()
ultrasound_stop_event = Event()

stop_images_event = Event()



four_servo_mode = True
    
####################

RANGE=15
NUM_TIMES = 5
DELAY = 0.005
SHIFT_VAL = 0
CAMERA = 1


def shutdown_scservo(SC_ID):

        # SCServo#1 torque
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SC_ID, ADDR_SCS_TORQUE_ENABLE, 0)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))
        

#####################
def close_running_threads():
    if is_pca9685_robot:
        pca.deinit()

    if is_gpio_robot:
        pwm0.set_PWM_dutycycle( servo0, 0 )
        pwm1.set_PWM_frequency( servo1, 0 )
        pwm2.set_PWM_frequency( servo2, 0 )
        pwm3.set_PWM_frequency( servo3, 0 )

    if has_lidar:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        #client.close()

    if is_scservo_robot:
        # Clear syncread parameter storage
        groupSyncRead.clearParam()
        
        shutdown_scservo(SCS1_ID)
        shutdown_scservo(SCS2_ID)
        shutdown_scservo(SCS3_ID)
        shutdown_scservo(SCS4_ID)
        
        # Close port
        portHandler.closePort()

    if has_ultrasound_trigger_on_pin_27 and has_ultrasound_echo_on_pin_22:
        ultrasound_thread.join()

#Register the function to be called on exit
atexit.register(close_running_threads)


#####################


@socketio.event
def connect():
    if has_lidar:
        global thread
        with thread_lock:
            if thread is None:
                thread = socketio.start_background_task(background_lidar_thread)
        emit('server_info', {'data': 'Connected!'})
    else:
        emit('server_info', {'data': '... No Lidar Connected.'})

    if has_ultrasound_trigger_on_pin_27 and has_ultrasound_echo_on_pin_22:
        global ultrasound_thread
        with ultrasound_thread_lock:
            if ultrasound_thread is None:
                ultrasound_thread = socketio.start_background_task(background_ultrasound_thread)


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

    if not has_picamera and not has_libcamera:
        session['CAMERA'] = 0

    if 'CAMERA_TYPE' not in session:
        if has_picamera:
            session['CAMERA_TYPE'] = 'picamera'
        elif has_libcamera:
            session['CAMERA_TYPE'] = 'picamera2'
        else:
            session['CAMERA_TYPE'] = 'none'


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
        sorted_saves = sorted(cleanedList,  reverse=True) 
    
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
        sorted_saves = sorted(cleanedList,  reverse=True) 
    
        print(sorted_saves)

        try:
            gripper_1 = np.load(sorted_saves[0] + '/angle_front_right_180.npy', allow_pickle=True)
            gripper_2 = np.load(sorted_saves[0] + '/angle_front_left_180.npy', allow_pickle=True)
            gripper_3 = np.load(sorted_saves[0] + '/angle_back_right_180.npy', allow_pickle=True)
            gripper_4 = np.load(sorted_saves[0] + '/angle_back_left_180.npy', allow_pickle=True)
        except:
            print("ERROR LOADING MOTION 1") 
       
    if 'FORWARD8' not in session:
        session['FORWARD8'] = None
    if 'BACKWARDS8' not in session:
        session['BACKWARDS8'] = None
    if 'LEFT8' not in session:
        session['LEFT8'] = None
    if 'RIGHT8' not in session:
        session['RIGHT8'] = None

    #Open defaults motions for 4 servos
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

    #Open defaults motions for 8 servos
    try: 
        with open("defaults/defaults8.txt") as f:
            print("OPENED DEFAULTS8 FILE")
            forward = f.readline().rstrip("\n") 
            backwards = f.readline().rstrip("\n") 
            left = f.readline().rstrip("\n") 
            right = f.readline().rstrip("\n") 
        
            session['FORWARD8'] = forward
            session['BACKWARDS8'] = backwards
            session['LEFT8'] = left
            session['RIGHT8'] = right
    
    except:
        print("ERROR:Missing defaults8 file?")


    #set defaults if not defined yet in Flask session.
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

    if 'g1_adjuster' not in session:
        session['g1_adjuster'] = 0
    
    if 'g2_adjuster' not in session:
        session['g2_adjuster'] = 0

    if 'g3_adjuster' not in session:
        session['g3_adjuster'] = 0

    if 'g4_adjuster' not in session:
        session['g4_adjuster'] = 0
        
    #Some debugging

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
           "CAMERA_TYPE":session['CAMERA_TYPE'],
           "FORWARD"    :session['FORWARD'],
           "BACKWARDS"  :session['BACKWARDS'],
           "LEFT"       :session['LEFT'],
           "RIGHT"      :session['RIGHT'],
           "FORWARD8"   :session['FORWARD8'],
           "BACKWARDS8" :session['BACKWARDS8'],
           "LEFT8"      :session['LEFT8'],
           "RIGHT8"     :session['RIGHT8'],
           "fr_adjust"  :session['front_right_adjuster'],
           "fl_adjust"  :session['front_left_adjuster'],
           "br_adjust"  :session['back_right_adjuster'],
           "bl_adjust"  :session['back_left_adjuster'],
           "g1_adjust"  :session['g1_adjuster'],
           "g2_adjust"  :session['g2_adjuster'],
           "g3_adjust"  :session['g3_adjuster'],
           "g4_adjust"  :session['g4_adjuster'],
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




def setSCServoPosition(SCS_ID, servo_angle):
    print("Setting Position ", SCS_ID, " to ", servo_angle)
    param_goal_position = [SCS_LOBYTE(servo_angle), SCS_HIBYTE(servo_angle)]

    scs_addparam_result = groupSyncWrite.addParam(SCS_ID, param_goal_position)
    if scs_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % SCS_ID)
        quit()


@app.route('/settoadjusters', methods=['POST', 'GET'])
def settoadjusters():

    if is_gpio_robot:
        pwm0.set_servo_pulsewidth(servo0, remap_to_pwm(90 + session['front_right_adjuster']))
        pwm1.set_servo_pulsewidth(servo1, remap_to_pwm(90 - session['front_left_adjuster']))
        pwm2.set_servo_pulsewidth(servo2, remap_to_pwm(90 + session['back_right_adjuster']))
        pwm3.set_servo_pulsewidth(servo3, remap_to_pwm(90 - session['back_left_adjuster']))


    elif is_pca9685_robot:
        servo0.angle = 90 + session['front_right_adjuster']
        servo1.angle = 90 - session['front_left_adjuster']
        servo2.angle = 90 + session['back_right_adjuster']
        servo3.angle = 90 - session['back_left_adjuster']
        servo4.angle = 90 + session['g1_adjuster']
        servo5.angle = 90 - session['g2_adjuster']
        servo6.angle = 90 + session['g3_adjuster']
        servo7.angle = 90 - session['g4_adjuster']

    elif is_scservo_robot:
        #Only handles 4 servos for Feetech robots
                
        servo0_angle = int(remap(90 + int(session['front_right_adjuster']), 0, 180, SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE))
        servo1_angle = int(remap(90 - int(session['front_left_adjuster']), 0, 180, SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE))
        servo2_angle = int(remap(90 + int(session['back_right_adjuster']), 0, 180, SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE))
        servo3_angle = int(remap(90 - int(session['back_left_adjuster']), 0, 180, SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE))
       
        setSCServoPosition(1, servo0_angle)
        setSCServoPosition(2, servo1_angle)
        setSCServoPosition(3, servo2_angle)
        setSCServoPosition(4, servo3_angle)
        
        # Syncwrite goal position
        scs_comm_result = groupSyncWrite.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
                
                # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()
             
    results = {'processed': 'true'}
    return jsonify(results)


@app.route('/stop', methods=['POST', 'GET'])
def stop():

    stop_robot_event.set()

    if is_gpio_robot:
        try:
            pwm0.set_servo_pulsewidth(servo0, remap_to_pwm(90 + session['front_right_adjuster']))
            pwm1.set_servo_pulsewidth(servo1, remap_to_pwm(90 - session['front_left_adjuster']))
            pwm2.set_servo_pulsewidth(servo2, remap_to_pwm(90 + session['back_right_adjuster']))
            pwm3.set_servo_pulsewidth(servo3, remap_to_pwm(90 - session['back_left_adjuster']))
        except:
            pwm0.set_servo_pulsewidth(servo0, remap_to_pwm(90))
            pwm1.set_servo_pulsewidth(servo1, remap_to_pwm(90))
            pwm2.set_servo_pulsewidth(servo2, remap_to_pwm(90))
            pwm3.set_servo_pulsewidth(servo3, remap_to_pwm(90))
    elif is_pca9685_robot:
        try:
            #Set to default positions
            servo0.angle = 90 + session['front_right_adjuster']
            servo1.angle = 90 - session['front_left_adjuster']
            servo2.angle = 90 + session['back_right_adjuster']
            servo3.angle = 90 - session['back_left_adjuster']
            servo4.angle = 90 + session['g1_adjuster']
            servo5.angle = 90 - session['g2_adjuster']
            servo6.angle = 90 + session['g3_adjuster']
            servo7.angle = 90 - session['g4_adjuster']
        except:
            #Adjusters not set
            servo0.angle = 90
            servo1.angle = 90
            servo2.angle = 90
            servo3.angle = 90
            servo4.angle = 90
            servo5.angle = 90
            servo6.angle = 90
            servo7.angle = 90

    elif is_scservo_robot:
         #Feetech servos are from 0 to 4096.  2048 for 90 degrees.
        setSCServoPosition(1, 2048)
        setSCServoPosition(2, 2048)
        setSCServoPosition(3, 2048)
        setSCServoPosition(4, 2048)
        
        # Syncwrite goal position
        scs_comm_result = groupSyncWrite.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
                
        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()


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




def updateDefault(motionName, defaultFileName):
    try: 
       
        defaultsArray = []

        with open(defaultFileName) as f:
            print("OPENED %s FILE" % defaultFileName)
            forward = f.readline().rstrip("\n") 
            backwards = f.readline().rstrip("\n") 
            left = f.readline().rstrip("\n") 
            right = f.readline().rstrip("\n") 

            if ("FORWARD" in motionName):        
                defaultsArray = [session[motionName], backwards, left, right]
            elif ("BACKWARDS" in motionName):        
                defaultsArray = [forward, session[motionName], left, right]
            elif ("LEFT" in motionName):        
                defaultsArray = [forward, backwards, session[motionName], right]
            elif ("RIGHT" in motionName):        
                defaultsArray = [forward, backwards, left, session[motionName]]

            npArray = np.array(defaultsArray)

            np.savetxt(defaultFileName, npArray, fmt="%s")

    except:
        print("Failed to update %s in %s" % motionName, defaultFileName)





@app.route('/updateForward', methods=['POST'])
def updateForward():
    data = request.get_json()
    session['FORWARD'] = data[0]['data']
    updateDefault("FORWARD", "defaults/defaults.txt")
    results = {'processed': 'true'}
    return jsonify(results)

@app.route('/updateBackwards', methods=['POST'])
def updateBackwards():
    data = request.get_json()
    session['BACKWARDS'] = data[0]['data']
    updateDefault("BACKWARDS", "defaults/defaults.txt")    
    results = {'processed': 'true'}
    return jsonify(results)

@app.route('/updateLeft', methods=['POST'])
def updateLeft():
    data = request.get_json()
    session['LEFT'] = data[0]['data']
    updateDefault("LEFT", "defaults/defaults.txt")    
    results = {'processed': 'true'}
    return jsonify(results)

@app.route('/updateRight', methods=['POST'])
def updateRight():
    data = request.get_json()
    session['RIGHT'] = data[0]['data']
    updateDefault("RIGHT", "defaults/defaults.txt")    
    results = {'processed': 'true'}
    return jsonify(results)



@app.route('/updateForward8', methods=['POST'])
def updateForward8():
    data = request.get_json()
    session['FORWARD8'] = data[0]['data']
    updateDefault("FORWARD8", "defaults/defaults8.txt")
    results = {'processed': 'true'}
    return jsonify(results)

@app.route('/updateBackwards8', methods=['POST'])
def updateBackwards8():
    data = request.get_json()
    session['BACKWARDS8'] = data[0]['data']
    updateDefault("BACKWARDS8", "defaults/defaults8.txt")
    results = {'processed': 'true'}
    return jsonify(results)

@app.route('/updateLeft8', methods=['POST'])
def updateLeft8():
    data = request.get_json()
    session['LEFT8'] = data[0]['data']
    updateDefault("LEFT8", "defaults/defaults8.txt")
    results = {'processed': 'true'}
    return jsonify(results)

@app.route('/updateRight8', methods=['POST'])
def updateRight8():
    data = request.get_json()
    session['RIGHT8'] = data[0]['data']
    updateDefault("RIGHT8", "defaults/defaults8.txt")
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



@app.route('/setg1', methods=['POST', 'GET'])
def setg1():
    data = request.get_json()
    session['g1_adjuster'] = int(data[0]['g1'])
    print(session['g1_adjuster'])
    results = {'processed': 'true'}
    return jsonify(results)

@app.route('/setg2', methods=['POST', 'GET'])
def setg2():
    data = request.get_json()
    session['g2_adjuster'] = int(data[0]['g2'])
    print(session['g2_adjuster'])
    results = {'processed': 'true'}
    return jsonify(results)

@app.route('/setg3', methods=['POST', 'GET'])
def setg3():
    data = request.get_json()
    session['g3_adjuster'] = int(data[0]['g3'])
    print(session['g3_adjuster'])
    results = {'processed': 'true'}
    return jsonify(results)

@app.route('/setg4', methods=['POST', 'GET'])
def setg4():
    data = request.get_json()
    session['g4_adjuster'] = int(data[0]['g4'])
    print(session['g4_adjuster'])
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
    sorted_saves = sorted(cleanedList,  reverse=True) 
    
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
    print("DIRS", listing)
    print("JSONIFIED:", jsonify(listing))
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

    with open(dirspath + "combo.txt", "w") as file_name:
        np.savetxt(file_name, npArray, fmt="%s")
    
    return index()


@app.route('/save', methods=['POST', 'GET'])
def save():
    print(request.form)

    front_right = request.form['front_right']
    front_left = request.form['front_left']
    back_right = request.form['back_right']
    back_left = request.form['back_left']
   
    
    
    try:
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

    with open(dirspath + 'angle_front_right_180.npy', "wb") as f:
        np.save(f, front_right_list) 
    
    with open(dirspath + 'angle_front_left_180.npy', "wb") as f:
        np.save(f, front_left_list) 
        
    with open(dirspath + 'angle_back_right_180.npy', "wb") as f:
        np.save(f, back_right_list) 

    with open(dirspath + 'angle_back_left_180.npy', "wb") as f:
        np.save(f, back_left_list) 


    with open(dirspath + 'angle_front_right_180.txt', "w") as f:
        np.savetxt(f, front_right_list, fmt='%d')
    
    with open(dirspath + 'angle_front_left_180.txt', "w") as f:
        np.savetxt(f, front_left_list, fmt='%d')
        
    with open(dirspath + 'angle_back_right_180.txt', "w") as f:
        np.savetxt(f, back_right_list, fmt='%d')

    with open(dirspath + 'angle_back_left_180.txt', "w") as f:
         np.savetxt(f, back_left_list, fmt='%d')

  
    return index()



@app.route('/fidget', methods=['POST', 'GET'])
def fidget():

    @copy_current_request_context
    def runFidgetTask():
        while True:
            if stop_robot_event.is_set(): 
                print(f'end {threading.current_thread().name} ')
                return
    
    
            path = './motions/'
            children = []
     
            for name in os.listdir(path):
                subpath = os.path.join(path, name)
                if os.path.isdir(subpath):
                    children.append(subpath)
    
            child = random.choice(children)
            print("Running ", child)
    
            checkpoints = glob(child + '/*')
            checkpoint_dirs = [x for x in checkpoints if os.path.isdir(x)]
            sorted_saves = sorted(checkpoint_dirs,  reverse=True)
    
            adjusters = [session['front_right_adjuster'], session['front_left_adjuster'], session['back_right_adjuster'], session['back_left_adjuster']]
            front_right, front_left, back_right, back_left = load_and_validate_motion(sorted_saves[0], adjusters)
            runadjustedmotiondirect([front_right, front_left, back_right, back_left], False, False)  # don't extract dict, don't run as thread
    


    stop_robot_event.clear()
    t = Thread(target=runFidgetTask, args=(), daemon=True)  
    t.start()
    results = {'processed': 'false'}
    return jsonify(results)



def validate_motion(motions, adjuster_array):
            
    front_right = np.array(motions[0]) + int(adjuster_array[0])
    front_left = np.array(motions[1]) + int(adjuster_array[1])
    back_right = np.array(motions[2]) + int(adjuster_array[2])
    back_left = np.array(motions[3]) + int(adjuster_array[3])
                
    front_right[front_right > 179] = 179
    front_left[front_left > 179] = 179
    back_right[back_right > 179] = 179
    back_left[back_left > 179] = 179

    front_right[front_right < 1] = 1
    front_left[front_left < 1] = 1
    back_right[back_right < 1] = 1
    back_left[back_left < 1] = 1

    front_left = 180-front_left
    back_left = 180-back_left
        
    min_len = min(len(front_right), len(front_left), len(back_right), len(back_left))
    min_len = min(min_len, 130)
    front_right = front_right[:min_len]
    front_left = front_left[:min_len]
    back_right = back_right[:min_len]
    back_left = back_left[:min_len]
    
    return front_right, front_left, back_right, back_left
            

def load_and_validate_motion(directory, adjuster_array):
            
    front_right = np.load(directory + '/angle_front_right_180.npy', allow_pickle=True)
    front_left = np.load(directory + '/angle_front_left_180.npy', allow_pickle=True)
    back_right = np.load(directory + '/angle_back_right_180.npy', allow_pickle=True)
    back_left = np.load(directory + '/angle_back_left_180.npy', allow_pickle=True)

    return validate_motion([front_right, front_left, back_right, back_left], adjuster_array)



@app.route('/runBrain', methods=['POST'])
def runBrain():

    @copy_current_request_context
    def runBrainTask():
        
        @copy_current_request_context
        def runMotionLogic(TOO_CLOSE, L, F, R):

            priority = 0
            
            # High Priority '0'
            if F < TOO_CLOSE:
                runMotionFromCategory("BACKWARDS")
                priority = 1
            if L < TOO_CLOSE:
                runMotionFromCategory("RIGHT")
                priority = 1
            if R < TOO_CLOSE:
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
            
            if priority == 0 and F > TOO_CLOSE:
                runMotionFromCategory("FORWARD")
            elif priority == 0 and F <= TOO_CLOSE:
                runMotionFromCategory("BACKWARDS")




        def poll_realsense():
            response = requests.post(url=realsense_url)
            print(response)
            return response




        @copy_current_request_context
        def runMotionFromCategory(category):

            try: 

                try:
                    four_servo_mode = session['MODE'] is None or session['MODE'] == "Four" 
                except:
                    session['MODE'] = "Four"
                    four_servo_mode = True
            
                print ("Mode:", four_servo_mode)
               
            
                if four_servo_mode:
                    
                    category_dir = "./motions/" + session[category] 
                    checkpoints = glob(category_dir + '/*')
                    checkpoint_dirs = [x for x in checkpoints if os.path.isdir(x)]
                    sorted_saves = sorted(checkpoint_dirs,  reverse=True) 
                    
                    adjusters = [session['front_right_adjuster'], session['front_left_adjuster'], session['back_right_adjuster'], session['back_left_adjuster']]
                    front_right, front_left, back_right, back_left = load_and_validate_motion(sorted_saves[0], adjusters)
                    
                    if is_gpio_robot:
                        for i in range(len(front_right)):
                            if stop_robot_event.is_set(): 
                                print(f'end {threading.current_thread().name} ')
                                return
                    
                            pwm0.set_servo_pulsewidth(servo0, remap_to_pwm(int(front_right[i])))
                            pwm1.set_servo_pulsewidth(servo1, remap_to_pwm(int(front_left[i])))
                            pwm2.set_servo_pulsewidth(servo2, remap_to_pwm(int(back_right[i])))
                            pwm3.set_servo_pulsewidth(servo3, remap_to_pwm(int(back_left[i])))
                            
                            DELAY = float(session['DELAY'])
                            time.sleep(DELAY)
                    if is_pca9685_robot:
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
                            
                            
                            
                else:  #8 servos
                 
                    category_dir = "./motions8/" + session[category] 
                    checkpoints = glob(category_dir + '/*')
                    checkpoint_dirs = [x for x in checkpoints if os.path.isdir(x)]
                    sorted_saves = sorted(checkpoint_dirs,  reverse=True) 
                    
                    #TODO
                    
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
                    
                    
                    category_dir = "./motions/" + comboArray[0]
                    checkpoints = glob(category_dir + '/*')
                    checkpoint_dirs = [x for x in checkpoints if os.path.isdir(x)]
                    sorted_saves = sorted(checkpoint_dirs,  reverse=True) 

                    adjusters = [session['front_right_adjuster'], session['front_left_adjuster'], session['back_right_adjuster'], session['back_left_adjuster']]
                    front_right, front_left, back_right, back_left = load_and_validate_motion(sorted_saves[0], adjusters)
                    
                    #gripper
                    
                    category_dir = "./motions/" + comboArray[1]
                    checkpoints = glob(category_dir + '/*')
                    checkpoint_dirs = [x for x in checkpoints if os.path.isdir(x)]
                    sorted_saves = sorted(checkpoint_dirs,  reverse=True) 

                    adjusters = [session['g1_adjuster'], session['g2_adjuster'], session['g3_adjuster'], session['g4_adjuster']]
                    gripper_1, gripper_2, gripper_3, gripper_4 = load_and_validate_motion(sorted_saves[0], adjusters)
                    
                    if is_gpio_robot:
                        for i in range(len(front_right)):
                            if stop_robot_event.is_set(): 
                                print(f'end {threading.current_thread().name} ')
                                return
                            pwm0.set_servo_pulsewidth(servo0, remap_to_pwm(int(front_right[i])))
                            pwm1.set_servo_pulsewidth(servo1, remap_to_pwm(int(front_left[i])))
                            pwm2.set_servo_pulsewidth(servo2, remap_to_pwm(int(back_right[i])))
                            pwm3.set_servo_pulsewidth(servo3, remap_to_pwm(int(back_left[i])))
                            
                            DELAY = float(session['DELAY'])
                            time.sleep(DELAY)

                    elif is_pca9685_robot:
                        for i in range(len(front_right)):
                            if stop_robot_event.is_set(): 
                                print(f'end {threading.current_thread().name} ')
                                return
                    
                            servo0.angle = int(front_right[i])
                            servo1.angle = int(front_left[i])
                            servo2.angle = int(back_right[i])
                            servo3.angle = int(back_left[i])
                            servo4.angle = int(gripper_1[i])
                            servo5.angle = int(gripper_2[i])
                            servo6.angle = int(gripper_3[i])
                            servo7.angle = int(gripper_4[i])
                            
                            DELAY = float(session['DELAY'])
                            time.sleep(DELAY)
                            

            except:
                print(traceback.format_exc())
    
    
        #end runMotionFromCategory
    

        if (has_lidar):
            while True:

                if stop_robot_event.is_set():
                    print(f'end {threading.current_thread().name} ')
                    return

                try:
                    TOO_CLOSE = 0.2

                    L = float(mem.retrieveState('L'))
                    F = float(mem.retrieveState('F'))
                    R = float(mem.retrieveState('R'))

                    print (L, " ", F, " ", R)

                    runMotionLogic(TOO_CLOSE, L, F, R)

                except:
                    print(traceback.format_exc())
                    print("ERROR RETRIEVING LIDAR STATE")
                    results = {'processed': 'true'}
                    return jsonify(results)


        elif (has_realsense):
            response = poll_realsense()
            #For realsense, the server advises direction
            runMotionFromCategory(response)
             
   
        elif (has_arduino_at_115200):
        
            TOO_CLOSE=40
            while True:
    
                if stop_robot_event.is_set():
                    print(f'end {threading.current_thread().name} ')
                    return
               
                read_json = poll_arduino()
                try:
                    R = int(read_json['R'])
                    F = int(read_json['F'])
                    L = int(read_json['L'])
                except:
                    print("ERROR POLLING ARDUINO")
                    results = {'processed': 'true'}
                    return jsonify(results)
            
                runMotionLogic(TOO_CLOSE, L, F, R)
                

    stop_robot_event.clear()
    t = Thread(target=runBrainTask, args=(), daemon=True)  #check the syntax
    t.start()

    results = {'processed': 'true'}
    return jsonify(results)





@app.route('/runadjustedmotion', methods=['POST', 'GET'])
def runadjustedmotion():
    data = request.get_json()
    return runadjustedmotiondirect(data, True, True)

def runadjustedmotiondirect(data, needs_to_extract, as_thread):
    @copy_current_request_context
    def runMotionTask(data, needs_to_extract):
    
        front_right_update = data[0]
        front_left_update = data[1]
        back_right_update = data[2]
        back_left_update = data[3]
        
#        print(front_right_update) 
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
   
        if needs_to_extract:
            front_right = front_right_update['front_right']
            front_left = front_left_update['front_left']
            back_right = back_right_update['back_right']
            back_left = back_left_update['back_left']
        else:
            front_right = front_right_update
            front_left = front_left_update
            back_right = back_right_update
            back_left = back_left_update

    
        motions = [front_right, front_left, back_right, back_left]
        adjusters = [session['front_right_adjuster'], session['front_left_adjuster'], session['back_right_adjuster'], session['back_left_adjuster']]
        front_right, front_left, back_right, back_left = validate_motion(motions, adjusters)
            
        NUM_TIMES = int(session['NUM_TIMES'])
        r_front_right = np.tile(front_right, NUM_TIMES)
        r_front_left = np.tile(front_left, NUM_TIMES)
        r_back_right = np.tile(back_right, NUM_TIMES)
        r_back_left = np.tile(back_left, NUM_TIMES)
      
         
        if not four_servo_mode:
            gripper_1 = gripper_1_update['gripper_1']
            gripper_2 = gripper_2_update['gripper_2']
            gripper_3 = gripper_3_update['gripper_3']
            gripper_4 = gripper_4_update['gripper_4']

            motions = [gripper_1, gripper_2, gripper_3, gripper_4]
            adjusters = [session['g1_adjuster'], session['g2_adjuster'], session['g3_adjuster'], session['g4_adjuster']]
            gripper_1, gripper_2, gripper_3, gripper_4 = validate_motion(motions, adjusters)

            gripper_1 = np.tile(gripper_1, NUM_TIMES)
            gripper_2 = np.tile(gripper_2, NUM_TIMES)
            gripper_3 = np.tile(gripper_3, NUM_TIMES)
            gripper_4 = np.tile(gripper_4, NUM_TIMES)
    
        if is_gpio_robot:
            for i in range(len(r_front_right)):
                if stop_robot_event.is_set(): 
                    print(f'end {threading.current_thread().name} ')
                    return
                pwm0.set_servo_pulsewidth(servo0, remap_to_pwm(int(r_front_right[i])))
                pwm1.set_servo_pulsewidth(servo1, remap_to_pwm(int(r_front_left[i])))
                pwm2.set_servo_pulsewidth(servo2, remap_to_pwm(int(r_back_right[i])))
                pwm3.set_servo_pulsewidth(servo3, remap_to_pwm(int(r_back_left[i])))
                            
                DELAY = float(session['DELAY'])
                time.sleep(DELAY)

        elif is_pca9685_robot:
            for i in range(len(r_front_right)):
                if stop_robot_event.is_set(): 
                    print(f'end {threading.current_thread().name} ')
                    return
   
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


        elif is_scservo_robot:
            
            for index in range(len(r_front_right)):
                if stop_robot_event.is_set(): 
                    print(f'end {threading.current_thread().name} ')
                    return
 
                
                #need to map 0-2048 low and high to 0 to 180
                
                servo0_angle = int(remap(int(r_front_right[index]), 0, 180, SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE))
                servo1_angle = int(remap(int(r_front_left[index]), 0, 180, SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE))
                servo2_angle = int(remap(int(r_back_right[index]), 0, 180, SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE))
                servo3_angle = int(remap(int(r_back_left[index]), 0, 180, SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE))
               
                param_goal_position_1 = [SCS_LOBYTE(servo0_angle), SCS_HIBYTE(servo0_angle)]
             
                scs_addparam_result = groupSyncWrite.addParam(SCS1_ID, param_goal_position_1)
                if scs_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % SCS1_ID)
                    quit()

                param_goal_position_2 = [SCS_LOBYTE(servo1_angle), SCS_HIBYTE(servo1_angle)]

                scs_addparam_result = groupSyncWrite.addParam(SCS2_ID, param_goal_position_2)
                if scs_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % SCS2_ID)
                    quit()

                param_goal_position_3 = [SCS_LOBYTE(servo2_angle), SCS_HIBYTE(servo2_angle)]

                scs_addparam_result = groupSyncWrite.addParam(SCS3_ID, param_goal_position_3)
                if scs_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % SCS3_ID)
                    quit()

                param_goal_position_4 = [SCS_LOBYTE(servo3_angle), SCS_HIBYTE(servo3_angle)]

                scs_addparam_result = groupSyncWrite.addParam(SCS4_ID, param_goal_position_4)
                if scs_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % SCS4_ID)
                    quit()

                # Syncwrite goal position
                scs_comm_result = groupSyncWrite.txPacket()
                if scs_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
                
                # Clear syncwrite parameter storage
                groupSyncWrite.clearParam()
             
                DELAY = float(session['DELAY'])
                time.sleep(DELAY)

    stop_robot_event.clear()
    if as_thread:
        t = Thread(target=runMotionTask, args=(data, needs_to_extract), daemon=True)  
        t.start()
    else:
        runMotionTask(data, needs_to_extract)
    
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
    
    children = sorted(children, key=lambda d: d['text'])
    return children


@app.route('/images_kill_switch', methods=['POST', 'GET'])
def images_kill_switch():
    stop_images_event.set()
    results = {'processed': 'true'}

    return jsonify(results)

@app.route('/pics_thread', methods=['POST', 'GET'])
def takePicsWhenMovementDetected():
    
    @copy_current_request_context
    def runImagesTask():
        while True: 

            if stop_images_event.is_set():
                print(f'end {threading.current_thread().name} ')
                return
               
            F_val = mem.retrieveState('F')

            print(F_val)
            F = float(F_val)
            print(F)
            if (F < 150): 
                socketio.emit('server_info', {'data': 'Detection!'})
            
            time.sleep(5)
                
    stop_images_event.clear()
    t = Thread(target=runImagesTask, args=(), daemon=True)  
    t.start()

    results = {'processed': 'true'}
    return jsonify(results)

lastfile = "static/1.jpg"


def save_image():
    print("Capture still")
    timestr = time.strftime("%Y%m%d-%H%M%S")
    lastfile = "static/snap_" + timestr + ".jpg"
    if has_picamera:
        cam = picamera.PiCamera()
        cam.resolution = (640, 480)
        cam.start_preview()
        sleep(2)
        cam.capture(lastfile)
        cam.stop_preview()
        cam.close()
    elif has_libcamera:

        picam2 = Picamera2()
        config = picam2.create_still_configuration(main={"size": (640,480)})
        picam2.configure(config)
        picam2.start()
        
        np_array = picam2.capture_array()
        timestr = time.strftime("%Y%m%d-%H%M%S")
        lastfile = "static/snap_" + timestr + ".jpg"
        picam2.capture_file(lastfile)
        picam2.stop()
        picam2.close()
    
    return(lastfile)


@app.route('/snapshot')
def snapshot():
    return send_file(save_image())

#Not functional yet
@app.route('/socketsnapshot')
def socketsnapshot():
    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (640,480)})
    picam2.configure(config)
    picam2.start()
    
    np_array = picam2.capture_array()
    timestr = time.strftime("%Y%m%d-%H%M%S")
    lastfile = "static/snap_" + timestr + ".jpg"
    picam2.capture_file(lastfile)
    picam2.stop()
    picam2.close()
    socketio.emit('my-image-event', {'image_data': np_array.tolist()})

if __name__ == '__main__':
    socketio.run(app, debug=True, host='0.0.0.0')
