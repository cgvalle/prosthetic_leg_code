import os; os.system('clear')
from paho.mqtt import client as mqtt_client
from leg.models import features_v1
from leg.models.motor_pid import MotorPIDController
import numpy as np
import leg.parameters as p
from leg.aux_tools import Buffer
import time
import joblib


broker = p.BROKER_HOST
port = 1883
topic = "data"


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
        self._accel_step = 1  # extra degrees per consecutive same-direction prediction
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
        self._angle = np.clip(value, 0, 90)

    def on_message(self, client, userdata, msg):
        data = np.frombuffer(msg.payload, dtype=p.PRECISION)
        data = data.reshape(p.NUM_CHANNELS, -1)
        self.buffer.data = data

    def update(self):
        angle_list = []
        try:
            while True:
                tic = time.time()
                data = self.buffer.data

                emg = data[self.emg_idx, :]
                if self.emg_prepro is not None:
                    features, _ = self.emg_prepro(emg)
                    features = features.reshape(1, -1)

                    # replace inf with 0
                    features = np.where(np.isinf(features), 0, features)

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
        except KeyboardInterrupt:
            self.angle = 0
            if self.motor_control:
                self.motor.target = 0
            self.client.publish('marker', 0, qos=0)
            time.sleep(0.1)  # give the publish time to go out before disconnecting


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
