import os; os.system('clear')
from paho.mqtt import client as mqtt_client
from leg.models import features_v1
import numpy as np
import leg.parameters as p
from leg.aux_tools import Buffer
import time
import joblib
from threading import Thread


broker = p.BROKER_HOST
port = 1883
topic = "data"

DEVICE_AS5600 = 0x36
IN1 = 18
IN2 = 19
FREQ = 1000
MIN_SPEED = 35


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

    def __init__(self, kp=5.0, ki=0.10, kd=0.15, dt=0.01):
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
        self._pid = PID(kp, ki, kd)
        self._dt = dt
        self._running = True

        thread = Thread(target=self._loop, daemon=True)
        thread.start()

    def _read_angle(self):
        data = self._bus.read_i2c_block_data(DEVICE_AS5600, 0x0C, 2)
        raw = (data[0] << 8) | data[1]
        return raw * 360.0 / 4096.0

    @staticmethod
    def _angle_error(target, current):
        err = (target - current) % 360.0
        if err > 180.0:
            err -= 360.0
        return err

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

    def _loop(self):
        t_prev = time.time()
        while self._running:
            t_now = time.time()
            loop_dt = t_now - t_prev
            if loop_dt < self._dt:
                time.sleep(self._dt - loop_dt)
                continue
            current = self._read_angle()
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


class RealTimeInference:
    def __init__(self,
                 window=0.1,
                 emg_prepro=None,
                 emg_model=None,
                 emg_idx=[0, 1, 2, 3],
                 acc_idx=[8, 9, 10],
                 motor_control=True,
                 ):
        self.update_speed = 1/10  # seconds
        self.window = window      # seconds
        self.motor_control = motor_control

        self.emg_idx = emg_idx
        self.acc_idx = acc_idx

        if self.motor_control:
            self.motor = MotorPIDController()

        # Angle
        self._angle = 0
        self._streak = 0      # positive = consecutive up, negative = consecutive down
        self._base_step = 5
        self._accel_step = 5  # extra degrees per consecutive same-direction prediction
        self._max_step = 40   # cap

        # preprocesamiento
        self.emg_prepro = emg_prepro

        # modelo
        self.emg_model = emg_model
        if self.emg_model is not None:
            self.emg_model = joblib.load(emg_model)

        # Buffer
        self.buffer = Buffer(self.window, roll=True)

        # pre-compile emg_prepro
        if self.emg_prepro is not None:
            self.emg_prepro(self.buffer.data[self.emg_idx, :])

        # MQTT
        self.client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2, 'inference')
        self.client.on_connect = on_connect
        self.client.on_message = self.on_message
        self.client.connect(broker, port)
        self.client.subscribe(topic)
        self.client.loop_start()

        self.update()

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, value):
        self._angle = np.clip(value, 90, 175)

    def on_message(self, client, userdata, msg):
        data = np.frombuffer(msg.payload, dtype=p.PRECISION)
        data = data.reshape(p.NUM_CHANNELS, -1)
        self.buffer.data = data

    def update(self):
        angle_list = []
        while True:
            tic = time.time()
            data = self.buffer.data

            emg = data[self.emg_idx, :]
            if self.emg_prepro is not None:
                features, _ = self.emg_prepro(emg)
                features = features.reshape(1, -1)

            if self.emg_model is not None:
                prediction = self.emg_model.predict(features)[0]

                if prediction == 1:
                    self._streak = self._streak + 1 if self._streak > 0 else 1
                else:
                    self._streak = self._streak - 1 if self._streak < 0 else -1

                step = min(self._base_step + (abs(self._streak) - 1) * self._accel_step,
                           self._max_step)

                prev = self._angle
                if prediction == 1:
                    self.angle += step
                else:
                    self.angle -= step

                if self._angle == prev:
                    self._streak = 0

                angle_list.append(float(self._angle))
                if len(angle_list) > 10:
                    angle_list = angle_list[-10:]

                if self.motor_control:
                    self.motor.target = self._angle

                self.client.publish('marker', int(self._angle), qos=0)

            toc = time.time()
            time.sleep(np.max([self.update_speed - (toc - tic), 0]))


def on_connect(client, userdata, flags, rc, properties):
    print(f"Connected with result code {rc}")
    client.subscribe('data')


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--debug', action='store_true', help='Disable motor control')
    args = parser.parse_args()

    RealTimeInference(
        emg_prepro=features_v1,
        emg_model='model.pkl',
        motor_control=not args.debug,
    )
