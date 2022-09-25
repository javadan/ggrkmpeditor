# ggrkmpeditor
## GGR KMP Editor 

Backup for Gallus Gallus Roboticus project 
Kinetic Motion Primitives editor

Copyleft License

[mirandamoss.com/gallusgallusroboticus](https://mirandamoss.com/gallusgallusroboticus)


A basic robot control UI integrating with a few sensors and controllers.

Default movements can be saved for LEFT/RIGHT/FORWARD/BACKWARDS.

Tested on Rpi Zero W (32 bit) and Rpi 4 (64 bit).
Realsense functionality works with the ggrrealsense project.

Configure the settings dictionary in config.py.

Options:

```
    lidar : has RPLidar attached at /dev/ttyUSB0
    pca9685_robot : Robot is using PCA9685 16-channel 12-bit PWM controller
    scservo_robot : Robot is using Feetech SC servos via URT-1 controller
    arduino_115200 : Robot is using an Arduino, returning Left (L), Front (F), Right (R) distances over serial 
    realsense : Robot is using Realsense D4XX depth/rgb sensor
    realsense_url : URL to retrieve Realsense data
    picamera : Has 32-bit RPi 'raspi' camera attached 
    libcamera : Has 64-bit RPi 'libcamera' camera attached
    continuous_servo_on_pin_25 : For gripper robots, additional servo features (TODO)
    has_switches_on_pins_23_34 : For gripper robots, additional (2) switch inputs
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
    
    Connect browser to URL of robot service.
```

Features:

```
    - Toggle mode between 4 and 8 servo UI.
        * Generally, the idea is to configure and save motions in the 4 servo mode.
        * The 8 servo mode just stores the names of two 4 servo motions.
        
    - Use Adjusters if servo angles were not calibrated to 90 degrees, prior to robot assembly.
    
    - UI uses a wavetable editor, allowing shifting up and down, left and right, and scaling up and down.
    
    - UI shows saved motions, to load and save.
    
    - UI displays Lidar map if attached.
    
    - UI has run button, and kill switch.
    
    - UI has default motions for LEFT/RIGHT/FORWARD/BACKWARDS, set with the context menu (right click) on the motion list.
        * This is a prerequisite for the 'Turn on Brain' button. 
    
    - 'Turn on Brain' button uses either Lidar, Realsense, or Arduino to use distance information to decide its next motion.
        * Use kill switch to cancel.
        
    - Unfinished TODO: Scripting functionality.
    
```
