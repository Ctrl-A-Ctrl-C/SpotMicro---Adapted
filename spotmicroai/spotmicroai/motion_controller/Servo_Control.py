import RPi.GPIO as GPIO
from time import sleep

servoPIN = 2
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

def SetAngle(angle):
    duty = angle
    GPIO.output(servoPIN, True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(servoPIN, False)
    pwm.ChangeDutyCycle(0)
    print(duty)

pwm = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
pwm.start(5) # Initialization

while True:
    angle = float(input("Enter Angle: "))
    SetAngle(angle)

pwm.stop()
GPIO.cleanup()
