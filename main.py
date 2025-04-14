import RPi.GPIO as GPIO
import time

def map(value, inMin, inMax, outMin, outMax):
    return (outMax - outMin) * (value - inMin) / (inMax - inMin) + outMin

def setAngle(angle):      # make the servo rotate to specific angle (0-180 degrees)
    angle = max(0, min(180, angle))
    pulse_width = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE)
    pwm = map(pulse_width, 0, 20000, 0, 100)
    p.ChangeDutyCycle(pwm)#map the angle to duty cycle and output it


# Button and LED Stuff
LedPin = 20  # Set GPIO17 as LED pin
BtnPin = 21  # Set GPIO18 as button pin
Led_status = True

# Define a setup function for some setup
def setup():
    global p
    SERVO_MIN_PULSE = 500
    SERVO_MAX_PULSE = 2500

    ServoPin = 18
    GPIO.setmode(GPIO.BCM)       # Numbers GPIOs by BCM
    GPIO.setup(ServoPin, GPIO.OUT)   # Set ServoPin's mode is output
    GPIO.output(ServoPin, GPIO.LOW)  # Set ServoPin to low
    p = GPIO.PWM(ServoPin, 50)     # set Frequecy to 50Hz
    p.start(0)                     # Duty Cycle = 0
    # Set the GPIO modes to BCM Numbering
    # Set LedPin's mode to output,
    # and initial level to high (3.3v)
    GPIO.setup(LedPin, GPIO.OUT, initial=GPIO.HIGH)
    # Set BtnPin's mode to input,
    # and pull up to high (3.3V)
    GPIO.setup(BtnPin, GPIO.IN)


# Define a callback function for button callback
def swLed(ev=None):
    global Led_status
    # Switch led status(on-->off; off-->on)
    Led_status = not Led_status
    GPIO.output(LedPin, Led_status)
    if Led_status:
        print ('LED OFF...')
    else:
        print ('...LED ON')

# Define a function to rotate the servo
def rotateServo(ev=None):
    for i in range(0, 181, 5):   #make servo rotate from 0 to 180 deg
        setAngle(i)     # Write to servo
        time.sleep(0.002)
        
# Define a function to set open or close for the servo
def setOpenClose(ev=None):
    global open
    open = not open


# Define a main function for main process
def main():
# Set up a falling detect on BtnPin,
    # and callback function to swLed
    GPIO.add_event_detect(BtnPin, GPIO.FALLING, callback=setOpenClose)
    global prevState
    prevState = False
    while True:
        print('oh')
        if (prevState != open):
            if (open == True):
                prevState = True
                for i in range(0, 181, 5):   #make servo rotate from 0 to 180 deg
                    print("open true: setting angle to: ", i)
                    setAngle(i)     # Write to servo
                    time.sleep(0.002)
            elif (open == False):
                prevState = False
                for i in range(180, -1, -5): #make servo rotate from 180 to 0 deg
                    print("open false: setting angle to: ", i)
                    setAngle(i)
                    time.sleep(0.002)
        time.sleep(1)

# Define a destroy function for clean up everything after
# the script finished
def destroy():
    # Turn off LED
    GPIO.output(LedPin, GPIO.HIGH)
    # Release resource
    p.stop()
    GPIO.cleanup()

# If run this script directly, do:
if __name__ == '__main__':
    setup()
    try:
        main()
    # When 'Ctrl+C' is pressed, the program
    # destroy() will be executed.
    except KeyboardInterrupt:
        destroy()


def loop():
    while True:
        for i in range(0, 181, 5):   #make servo rotate from 0 to 180 deg
            setAngle(i)     # Write to servo
            time.sleep(0.002)
        time.sleep(1)
        for i in range(180, -1, -5): #make servo rotate from 180 to 0 deg
            setAngle(i)
            time.sleep(0.001)
        time.sleep(1)

