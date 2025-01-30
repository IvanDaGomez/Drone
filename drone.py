from picamera2 import Picamera2
import os
os.system("sudo pigpiod")
from gpiozero import AngularServo, DigitalOutputDevice, PWMOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory
import time
from drone.sensors.barometer import BMP180
from drone.sensors.infrarred_sensor import InfraredSensor
from drone.sensors.accerelerometer_gyro.MPU6050 import MPU6050

class Drone:
    def __init__(self, 
                servo_port, 
                infrarred_port,
                motor1_port, enablePin1_port, 
                #motor2_port, enablePin2_port,
                #motor3_port, enablePin3_port,
                #motor4_port, enablePin4_port
                ):
        self.camera = Picamera2()
        # Configure the preview
        preview_config = self.camera.create_preview_configuration()
        self.camera.configure(preview_config)
        self.camera.start()
        my_factory = PiGPIOFactory() 
        SERVO_DELAY_SEC = 0.001 
        myCorrection=0.0
        maxPW=(2.5+myCorrection)/1000
        minPW=(0.5-myCorrection)/1000
        self.servo =  AngularServo(servo_port,initial_angle=0,min_angle=0, max_angle=180,min_pulse_width=minPW,max_pulse_width=maxPW,pin_factory=my_factory)
        self.motor1 = DigitalOutputDevice(motor1_port)           # define L293D pin according to BCM Numbering
        self.enablePin1 = PWMOutputDevice(enablePin1_port,frequency=1000)
        #self.motor2 = DigitalOutputDevice(motor2_port)           # define L293D pin according to BCM Numbering
        #self.enablePin2 = PWMOutputDevice(enablePin2_port,frequency=1000)
        #self.motor3 = DigitalOutputDevice(motor3_port)           # define L293D pin according to BCM Numbering
        #self.enablePin3 = PWMOutputDevice(enablePin3_port,frequency=1000)
        #self.motor4 = DigitalOutputDevice(motor4_port)           # define L293D pin according to BCM Numbering
        #self.enablePin4 = PWMOutputDevice(enablePin4_port,frequency=1000)

        self.barometer = BMP180()

        self.infrarred = InfraredSensor(infrarred_port, pull_up=True, pull_up_down = )
        self.infrarred_active = False
        self.infrarred.when_reflect = self.on_reflect
        self.infrarred.when_no_reflect = self.on_no_reflect
        self.MPU6050 = MPU6050()
    
    def on_reflect(self):
        self.infrarred_active = True

    def on_no_reflect(self):
        self.infrarred_active = False

    def turn_motors_on(self, motor1_power = 0, motor2_power = 0, motor3_power = 0, motor4_power =0):
        # PID controller

        self.enablePin1.value = motor1_power / 100.0
        self.motor1.on()
        #self.enablePin2.value = motor2_power / 100.0
        #self.motor2.on()
        #self.enablePin3.value = motor3_power / 100.0
        #self.motor3.on()
        #self.enablePin4.value = motor4_power / 100.0
        #self.motor4.on()
    
    def rotate_servo(self, angle):
        self.servo.angle = angle
    def is_near_floor(self):
        return self.infrarred_active
    def get_stats(self):
        temperature = self.barometer.read_temperature()
        pressure = self.barometer.read_pressure()
        altitude = self.barometer.read_altitude()
        sea_level_pressure = self.barometer.read_sealevel_pressure()
        return temperature, altitude, pressure, sea_level_pressure
    def get_gyro_values(self):
        gyro_values = self.MPU6050.get_rotation()
        return gyro_values  #a list of [x, y, z]
    def get_accelerometer_values(self):
        accelerometer_values = self.MPU6050.get_acceleration()
        return accelerometer_values
    def close(self):
        self.servo.close()
        self.camera.stop()
        os.system("sudo killall pigpiod")