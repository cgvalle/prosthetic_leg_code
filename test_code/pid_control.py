import RPi.GPIO as GPIO
import time
from smbus2 import SMBus

# ── Motor (pins match pwm.py) ─────────────────────────────────────────────────
IN1 = 13
IN2 = 18
FREQ = 10000

GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

pwm1 = GPIO.PWM(IN1, FREQ)
pwm2 = GPIO.PWM(IN2, FREQ)
pwm1.start(0)
pwm2.start(0)

# ── Encoder (AS5600 via I2C, matches encoder.py) ──────────────────────────────
DEVICE_AS5600 = 0x36
bus = SMBus(1)


ESTIRADA = 240   # fully extended raw angle
CONTRAIDA = 170  # fully contracted raw angle
ANGLE_MAX = 90   # degrees, fully contracted

def _clamp(x, lo=0.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x

def read_angle():
    """Return knee angle in degrees [0, 90] (0 = fully extended, 90 = fully contracted)."""
    data = bus.read_i2c_block_data(DEVICE_AS5600, 0x0C, 2)
    raw = (data[0] << 8) | data[1]
    raw_deg = raw * 360.0 / 4096.0
    return _clamp((raw_deg - ESTIRADA) / (CONTRAIDA - ESTIRADA)) * ANGLE_MAX

def angle_error(target, current):
    """Signed error in degrees. Range is linear [0, 90] so no wraparound needed."""
    return target - current

# ── Motor driver ──────────────────────────────────────────────────────────────
MIN_SPEED = 100   # minimum duty cycle to overcome static friction (tune this)

def motor_drive(output):
    """
    Drive motor with signed duty cycle.
    output > 0 → forward, output < 0 → backward, 0 → stop.
    Magnitude is clamped to [MIN_SPEED, 100] when non-zero.
    """
    output = max(-100.0, min(100.0, output))
    if abs(output) < 1.0:
        pwm1.ChangeDutyCycle(0)
        pwm2.ChangeDutyCycle(0)
        return
    speed = max(MIN_SPEED, abs(output))
    if output > 0:
        pwm1.ChangeDutyCycle(0)
        pwm2.ChangeDutyCycle(speed)
    else:
        pwm1.ChangeDutyCycle(speed)
        pwm2.ChangeDutyCycle(0)

def motor_stop():
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)

# ── PID controller ────────────────────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, integral_limit=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self._integral = 0.0
        self._prev_error = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = None

    def compute(self, error, dt):
        self._integral += error * dt
        # Anti-windup: clamp integral contribution
        self._integral = max(-self.integral_limit, min(self.integral_limit, self._integral))

        derivative = 0.0
        if self._prev_error is not None and dt > 0:
            derivative = (error - self._prev_error) / dt
        self._prev_error = error

        return self.kp * error + self.ki * self._integral + self.kd * derivative

# ── Position control loop ─────────────────────────────────────────────────────
def run_to_angle(target_angle, kp=7.0, ki=0.10, kd=0.20,
                 tolerance=1.5, timeout=10.0, dt=0.01):
    """
    Drive the motor until the knee reaches target_angle (degrees).

    Args:
        target_angle: Desired knee angle in [0, 90] (0 = extended, 90 = contracted).
        kp, ki, kd:   PID gains (tune for your motor + load).
        tolerance:    Acceptable error in degrees before declaring success.
        timeout:      Maximum run time in seconds.
        dt:           Control loop period in seconds.
    """
    pid = PID(kp, ki, kd)
    t_start = time.time()
    t_prev = t_start

    print(f"Moving to {target_angle:.1f}°  (tolerance ±{tolerance}°, timeout {timeout}s)")

    try:
        while True:
            t_now = time.time()
            elapsed = t_now - t_start
            loop_dt = t_now - t_prev

            if loop_dt < dt:
                time.sleep(dt - loop_dt)
                continue

            current = read_angle()
            error = angle_error(target_angle, current)
            output = pid.compute(error, loop_dt)

            print(f"  t={elapsed:5.2f}s  current={current:6.2f}°  "
                  f"error={error:+6.2f}°  output={output:+6.1f}")

            if abs(error) <= tolerance:
                motor_stop()
                print(f"Done. Reached {current:.2f}° (target {target_angle:.1f}°)")
                break

            if elapsed > timeout:
                motor_stop()
                print(f"Timeout after {timeout}s. Last angle: {current:.2f}°")
                break

            motor_drive(output)
            t_prev = t_now

    except KeyboardInterrupt:
        motor_stop()
        print("Interrupted.")

# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    try:
        for target in [0, 90, 0]:
            #target = float(input("Enter target knee angle (0–90): "))
            # target = 45.0   # mid-flex, degrees [0=extended, 90=contracted]
            run_to_angle(target, kp=10.0, ki=10, kd=0.15)
            motor_stop()


    finally:
        motor_stop()
        pwm1.stop()
        pwm2.stop()
        GPIO.cleanup()
