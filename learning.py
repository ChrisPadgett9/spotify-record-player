import RPi.GPIO as GPIO
import time

# GPIO Pins
ENA = 22  # PWM
IN1 = 17
IN2 = 27

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# PWM setup
pwm = GPIO.PWM(ENA, 1000)  # 1 kHz frequency
pwm.start(0)  # Start with 0% duty cycle

def motor_forward(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(speed)

def motor_backward(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)

def stop_motor():
    pwm.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

try:
    while True:
        for speed in range(0, 101, 20):  # Ramp up
            print(f"Speed: {speed}%")
            motor_forward(speed)
            time.sleep(1)
        stop_motor()
        time.sleep(2)
        for speed in range(100, -1, -20):  # Ramp down
            print(f"Speed: {speed}%")
            motor_backward(speed)
            time.sleep(1)
        stop_motor()
        time.sleep(2)
except KeyboardInterrupt:
    print("Stopping motor")
    stop_motor()
    GPIO.cleanup()
