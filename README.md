# ggrkmpeditor
GGR KMP Editor 

Backup for Gallus Gallus Roboticus project 
Kinetic Motion Primitives editor

mirandamoss.com/gallusgallusroboticus

A basic robot control UI integrating with a few sensors and controllers.

Tested on Rpi Zero W (32 bit) and Rpi 4 (64 bit)

Configure the settings dictionary in config.py.

Options:
    lidar : has RPLidar attached at /dev/ttyUSB0
    pca9685_robot : Robot is using PCA9685 16-channel 12-bit PWM controller
    scservo_robot : Robot is using Feetech SC servos via URT-1 controller
    arduino_115200 : Robot is using an Arduino, returning Left (L), Front (F), Right (R) distances over serial 
    realsense : Robot is using Realsense D4XX depth/rgb sensor
    realsense_url : URL to retrieve Realsense data
    picamera : Has RPi camera attached. (Deprecated in newest RPi Bullseye, in favour of libcamera. (TODO)) 
    
    
    
