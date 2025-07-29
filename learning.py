import RPi.GPIO as GPIO
import time
import threading

# GPIO Pins
ENA = 22  # PWM
IN1 = 17
IN2 = 27
BTNPIN = 21  # Set GPIO21 as button pin

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(BTNPIN, GPIO.IN)  # Use pull-up for button

# PWM setup
pwm = GPIO.PWM(ENA, 1250)  # 1 kHz frequency
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

motor_running = threading.Event()

def motor_loop():
    while True:
        motor_running.wait()  # Wait until the event is set
        for speed in range(0, 101, 20):  # Ramp up
            if not motor_running.is_set():
                break
            print(f"Speed: {speed}%")
            motor_forward(speed)
            time.sleep(3)
        stop_motor()
        time.sleep(3)
        for speed in range(100, -1, -20):  # Ramp down
            if not motor_running.is_set():
                break
            print(f"Speed: {speed}%")
            motor_backward(speed)
            time.sleep(3)
        stop_motor()
        time.sleep(3)

def button_callback(channel):
    if motor_running.is_set():
        print("Stopping motor via button")
        motor_running.clear()
        stop_motor()
    else:
        print("Starting motor via button")
        motor_running.set()

GPIO.add_event_detect(BTNPIN, GPIO.FALLING, callback=button_callback, bouncetime=300)

try:
    t = threading.Thread(target=motor_loop, daemon=True)
    t.start()
    while True:
        time.sleep(0.1)  # Main thread idle
except KeyboardInterrupt:
    print("Stopping motor")
    stop_motor()
    GPIO.cleanup()
