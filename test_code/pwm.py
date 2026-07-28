import RPi.GPIO as GPIO
import time

IN1 = 18  # Forward PWM pin
IN2 = 13  # Backward PWM pin

FREQ = 10000  # 1 kHz

GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

pwm1 = GPIO.PWM(IN1, FREQ)
pwm2 = GPIO.PWM(IN2, FREQ)

pwm1.start(0)
pwm2.start(0)

def motor_forward(speed):
    """speed: 0–100 (percent)"""
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(0)

def motor_backward(speed):
    """speed: 0–100 (percent)"""
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(speed)

def motor_stop():
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)

# Example
try: 
    duration = 1
    # LEFT: contracción (amarillo)
    #motor_forward(100)   # 75% speed forward
    #time.sleep(duration)
    motor_stop()


    # RIGHT: extension (naranjo)
    motor_backward(100)  # 50% speed backward
    time.sleep(duration)
    motor_stop()

    # keyboard cancell
except KeyboardInterrupt:
    motor_stop()

    pass
# Cleanup
pwm1.stop()
pwm2.stop()
GPIO.cleanup()

# estirada es 240
# contracción es 170