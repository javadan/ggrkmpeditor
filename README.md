# ggrkmpeditor
## GGR KMP Editor 

Gallus Gallus Roboticus 
Kinetic Motion Primitives editor

Four Servo example:
![Four servo example](https://github.com/javadan/ggrkmpeditor/blob/master/GGR4.png "GGR4")

Eight Servo example:
![Eight servo example](https://github.com/javadan/ggrkmpeditor/blob/master/GGR8.png "GGR8")



Copyleft License

[mirandamoss.com/gallusgallusroboticus](https://mirandamoss.com/gallusgallusroboticus)


A basic robot control UI integrating with a few sensors and controllers.

Default movements can be saved for LEFT/RIGHT/FORWARD/BACKWARDS.

Tested on Rpi Zero W (32 bit) and Rpi 4 (64 bit).
Realsense functionality works with the [ggrrealsense](https://github.com/javadan/ggrrealsense) project.  Tested with D455 depth camera.
Lidar used is RPLidar A1M8.  

Configure the settings dictionary in config.py.

Options:

```
    lidar : has RPLidar attached at /dev/ttyUSB0
    pca9685_robot : Robot is using PCA9685 16-channel 12-bit PWM controller
    scservo_robot : Robot is using Feetech SC servos via URT-1 controller
    gpio_on_5_6_13_19_robot : Robot has servos directly connected to (BCM pins) 5, 6, 13, 19
    arduino_115200 : Robot is using an Arduino, returning Left (L), Front (F), Right (R) distances over serial 
    realsense : Robot is using Realsense D4XX depth/rgb sensor
    realsense_url : URL to retrieve Realsense data
    picamera : Has 32-bit RPi 'raspi' camera attached 
    libcamera : Has 64-bit RPi 'libcamera' camera attached
    ultrasound_echo_on_pin_22 : Ultrasound sensor echo pin, for taking pictures on detection
    ultrasound_trigger_on_pin_27 : Ultrasound sensor trigger pin, for taking pictures on detection
    continuous_servo_on_pin_25 : For gripper robots, additional servo features (TODO)
    has_switches_on_pins_23_34 : For gripper robots, additional (2) switch inputs (used for preventing continuous servo death)
```
    
Hardware setup:

```
    Servo motions assume forward facing servos.  Therefore the left servo angles are subtracted from 180 degrees.

```
 
Setup (Note: For global installation, as this is the only program running on the Rpi.  Use venv if virtual environment required)

```
    chmod 755 pi_boot.sh
    ./pi_boot
    pip3 install -r requirements.txt
    
    chmod 755 run.sh
    ./run.sh
    
    Connect browser to URL of robot service. Typically 192.168.xxx.xxx:8000.
```

Features:

```
    - Toggle mode between 4 and 8 servo UI.
        * Generally, the idea is to configure and save motions in the 4 servo mode.
        * The 8 servo mode just stores the names of two 4 servo motions.
        
    - Use Adjusters if servo angles were not calibrated to 90 degrees, prior to robot assembly.
    
    - UI uses a wavetable editor, allowing shifting up and down, left and right, and scaling up and down, stretching left and right.
    
    - UI shows saved motions, to load and save.  Hover-over shows load preview.
    
    - UI displays Lidar map if attached.
    
    - UI has run button, and kill switch.
    
    - UI has a fidget feature, to run saved motions randomly.  
    
    - UI takes pictures, and can run a thread to take pictures on ultrasound detection
        * Note that there is currently a bug in picamera2 logging, which causes this thread to crash after a few minutes (TODO)
    
    - UI has default motions for LEFT/RIGHT/FORWARD/BACKWARDS, set with the context menu (right click) on the motion list.
        * This is a prerequisite for the 'Turn on Brain' button. 
    
    - 'Turn on Brain' button uses either Lidar, Realsense, or Arduino to use distance information to decide its next motion.
        * Use kill switch to cancel.
        
    - Unfinished TODO: Scripting functionality.
    
```
