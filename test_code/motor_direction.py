import RPi.GPIO as GPIO
import time

IN1 = 18
IN2 = 19
FREQ = 1000

GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

pwm1 = GPIO.PWM(IN1, FREQ)
pwm2 = GPIO.PWM(IN2, FREQ)
pwm1.start(0)
pwm2.start(0)

def forward(speed=100):
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(0)

def backward(speed=100):
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(speed)

def stop():
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)

try:
    print("Motor direction control")
    print("  f = forward")
    print("  b = backward")
    print("  s = stop")
    print("  q = quit")

    while True:
        cmd = input("Command: ").strip().lower()
        if cmd == "f":
            forward()
            print("Moving forward")
        elif cmd == "b":
            backward()
            print("Moving backward")
        elif cmd == "s":
            stop()
            print("Stopped")
        elif cmd == "q":
            break
        else:
            print("Unknown command")

except KeyboardInterrupt:
    pass

stop()
pwm1.stop()
pwm2.stop()
GPIO.cleanup()
