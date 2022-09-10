from flask import Flask, render_template, request, jsonify, session
import requests
from flask_session import Session
#from flask_autoindex import AutoIndex
from datetime import datetime
import os.path
from glob import glob
import os
import atexit
import numpy as np
import json

import time

from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685




####################

i2c = busio.I2C(SCL, SDA)

pca = PCA9685(i2c)
pca.frequency = 50

servo0 = servo.Servo(pca.channels[0])
servo1 = servo.Servo(pca.channels[1])
servo2 = servo.Servo(pca.channels[2])
servo3 = servo.Servo(pca.channels[3])

servo0.angle = 90
servo1.angle = 90
servo2.angle = 90
servo3.angle = 90


NUM_TIMES = 5
DELAY = 0.005

#####################
def close_running_threads():
    pca.deinit()

#Register the function to be called on exit
atexit.register(close_running_threads)


#####################

#ppath = "/home/pi/webapp"


app = Flask(__name__)

#AutoIndex(app, browse_root=ppath)  

SESSION_TYPE = 'filesystem'
app.config.from_object(__name__)
Session(app)



#session['NUM_TIMES'] = NUM_TIMES
#session['DELAY'] = DELAY


@app.route('/')
def index():
    
    front_right = np.load('./motions/walk/angle_front_right_180.npy')
    front_left = np.load('./motions/walk/angle_front_left_180.npy')
    back_right = np.load('./motions/walk/angle_back_right_180.npy')
    back_left = np.load('./motions/walk/angle_back_left_180.npy')

    print (front_right)

    session['NUM_TIMES'] = NUM_TIMES
    session['DELAY'] = DELAY
    session['MOTION'] = 'walk'

    print ("INDEX MOTION: " + session['MOTION'])
    return render_template('index.html',
        leg_data=
        {"MOTION"     :session['MOTION'],
         "x"          :json.dumps(np.arange(front_right.size).tolist()),
         "front_right":json.dumps(front_right.tolist()),
         "front_left" :json.dumps(front_left.tolist()),
         "back_right" :json.dumps(back_right.tolist()),
         "back_left"  :json.dumps(back_left.tolist())})


@app.route('/stop', methods=['POST', 'GET'])
def stop():
    servo0.angle = 90
    servo1.angle = 90
    servo2.angle = 90
    servo3.angle = 90
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


@app.route('/getdirs', methods=['POST', 'GET'])
def getdirs():
    data = request.get_json()
    print (data)
    listing = get_directory_listing("./motions")
    print (jsonify(listing))
    return jsonify(listing)


@app.route('/load', methods=['POST', 'GET'])
def load():
    print(request.form)
    load_id = request.form['id']
    print (load_id)

    print('./motions/' + load_id)
    
    checkpoints = glob('./motions/' + load_id + '/*')
    print(checkpoints)

    cleanedList = [x for x in checkpoints if os.path.isdir(x)]
    sorted_saves = sorted(cleanedList, key=lambda x: x[1], reverse=True) 

    print(sorted_saves)

    front_right = np.load(sorted_saves[0] + '/angle_front_right_180.npy', allow_pickle=True)
    front_left = np.load(sorted_saves[0] + '/angle_front_left_180.npy', allow_pickle=True)
    back_right = np.load(sorted_saves[0] + '/angle_back_right_180.npy', allow_pickle=True)
    back_left = np.load(sorted_saves[0] + '/angle_back_left_180.npy', allow_pickle=True)


    print (front_right)
    session['NUM_TIMES'] = NUM_TIMES
    session['DELAY'] = DELAY
    session['MOTION'] = load_id

    print ("MOTION: " + session['MOTION'])

    return render_template('index.html',
        leg_data=
        {"MOTION"     :session['MOTION'],
         "x"          :json.dumps(np.arange(front_right.size).tolist()),
         "front_right":json.dumps(front_right.tolist()),
         "front_left" :json.dumps(front_left.tolist()),
         "back_right" :json.dumps(back_right.tolist()),
         "back_left"  :json.dumps(back_left.tolist())})



@app.route('/save', methods=['POST', 'GET'])
def save():
    print(request.form)

    front_right = request.form['front_right']
    front_left = request.form['front_left']
    back_right = request.form['back_right']
    back_left = request.form['back_left']

    print(front_right)

    save_id = request.form['id']
    print (save_id)

    front_right_list = json.loads('[' + front_right + ']')
    front_left_list = json.loads('[' + front_left + ']')
    back_right_list = json.loads('[' + back_right + ']')
    back_left_list = json.loads('[' + back_left + ']')

    print(front_right_list)

    print('./motions/' + save_id)

    date = datetime.now().strftime("%Y%m%d%I%M%S%f")

    #dirs must exist first
    dirspath = './motions/' + save_id + '/' + date + '/' 
    os.makedirs(dirspath)

    np.save(dirspath + 'angle_front_right_180.npy', front_right_list) 
    np.save(dirspath + 'angle_front_left_180.npy', front_left_list) 
    np.save(dirspath + 'angle_back_right_180.npy', back_right_list) 
    np.save(dirspath + 'angle_back_left_180.npy', back_left_list) 

    return render_template('index.html',
        leg_data=
        {"MOTION"     :session['MOTION'],
         "x"          :json.dumps(np.arange(len(front_right_list)).tolist()),
         "front_right":json.dumps(front_right_list),
         "front_left" :json.dumps(front_left_list),
         "back_right" :json.dumps(back_right_list),
         "back_left"  :json.dumps(back_left_list)})


@app.route('/runmotion', methods=['POST', 'GET'])
def runmotion():
    data = request.get_json()
    front_right_update = data[0]
    front_left_update = data[1]
    back_right_update = data[2]
    back_left_update = data[3]

    print(front_right_update['front_right'])
    print(front_left_update['front_left'])
    print(back_right_update['back_right'])
    print(back_left_update['back_left'])

    front_right = front_right_update['front_right']
    front_left = front_left_update['front_left']
    back_right = back_right_update['back_right']
    back_left = back_left_update['back_left']


    results = {'processed': 'true'}
   
    NUM_TIMES = int(session['NUM_TIMES'])
    print(NUM_TIMES)
    r_front_right = np.tile(front_right, NUM_TIMES)
    r_front_left = np.tile(front_left, NUM_TIMES)
    r_back_right = np.tile(back_right, NUM_TIMES)
    r_back_left = np.tile(back_left, NUM_TIMES)

    r_front_right = 180-r_front_right
    r_back_right = 188-r_back_right


    for i in range(len(r_front_right)):
        servo0.angle = int(r_front_right[i])
        servo1.angle = int(r_front_left[i])
        servo2.angle = int(r_back_right[i])
        servo3.angle = int(r_back_left[i])
        DELAY = float(session['DELAY'])
        time.sleep(DELAY)


    servo0.angle = 90
    servo1.angle = 90
    servo2.angle = 90
    servo3.angle = 90

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





if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')
