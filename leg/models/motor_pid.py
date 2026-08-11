import time
from threading import Thread

DEVICE_AS5600 = 0x36
IN1 = 13
IN2 = 18
FREQ = 10000
MIN_SPEED = 100

ESTIRADA = 235   # fully extended raw angle
CONTRAIDA = 170  # fully contracted raw angle
ANGLE_MAX = 90   # degrees, fully contracted


def _clamp(x, lo=0.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x


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
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral + error * dt))
        derivative = ((error - self._prev_error) / dt
                      if self._prev_error is not None and dt > 0 else 0.0)
        self._prev_error = error
        return self.kp * error + self.ki * self._integral + self.kd * derivative


class MotorPIDController:

    def __init__(self, kp=0.5, ki=0.01, kd=0.01, dt=0.01):
        import RPi.GPIO as GPIO
        from smbus2 import SMBus

        self._GPIO = GPIO
        self._bus = SMBus(1)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(IN1, GPIO.OUT)
        GPIO.setup(IN2, GPIO.OUT)
        self._pwm1 = GPIO.PWM(IN1, FREQ)
        self._pwm2 = GPIO.PWM(IN2, FREQ)
        self._pwm1.start(0)
        self._pwm2.start(0)

        self._target = self._read_angle()
        self._current = self._target
        self._pid = PID(kp, ki, kd)
        self._dt = dt
        self._running = True

        thread = Thread(target=self._loop, daemon=True)
        thread.start()

    def _read_angle(self):
        """Return knee angle in degrees [0, 90] (0 = fully extended, 90 = fully contracted)."""
        data = self._bus.read_i2c_block_data(DEVICE_AS5600, 0x0C, 2)
        raw = (data[0] << 8) | data[1]
        raw_deg = raw * 360.0 / 4096.0
        return _clamp((raw_deg - ESTIRADA) / (CONTRAIDA - ESTIRADA)) * ANGLE_MAX

    @staticmethod
    def _angle_error(target, current):
        """Signed error in degrees. Range is linear [0, 90] so no wraparound needed."""
        return target - current

    def _drive(self, output):
        output = max(-100.0, min(100.0, output))
        if abs(output) < 1.0:
            self._pwm1.ChangeDutyCycle(0)
            self._pwm2.ChangeDutyCycle(0)
            return
        speed = max(MIN_SPEED, abs(output))
        if output > 0:
            self._pwm1.ChangeDutyCycle(0)
            self._pwm2.ChangeDutyCycle(speed)
        else:
            self._pwm1.ChangeDutyCycle(speed)
            self._pwm2.ChangeDutyCycle(0)

    def _stop(self):
        self._pwm1.ChangeDutyCycle(0)
        self._pwm2.ChangeDutyCycle(0)

    @property
    def target(self):
        return self._target

    @target.setter
    def target(self, value):
        self._target = float(value)

    @property
    def current(self):
        """Last angle read by the control loop, in degrees [0, 90]."""
        return self._current

    def _loop(self):
        t_prev = time.time()
        while self._running:
            t_now = time.time()
            loop_dt = t_now - t_prev
            if loop_dt < self._dt:
                time.sleep(self._dt - loop_dt)
                continue
            current = self._read_angle()
            self._current = current
            error = self._angle_error(self._target, current)
            output = self._pid.compute(error, loop_dt)
            self._drive(output)
            print(f'target: {self._target:.1f}°  current: {current:.1f}°  '
                  f'error: {error:+.1f}°  output: {output:+.1f}')
            t_prev = t_now

    def stop(self):
        self._running = False
        self._stop()
        self._pwm1.stop()
        self._pwm2.stop()
        self._GPIO.cleanup()
