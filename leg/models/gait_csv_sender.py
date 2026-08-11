import os; os.system('clear')
import time
import numpy as np
import pandas as pd
from paho.mqtt import client as mqtt_client
from leg.models.motor_pid import MotorPIDController
import leg.parameters as p


broker = p.BROKER_HOST
port = 1883
topic = "data"

CSV_PATH = os.path.join(os.path.dirname(__file__), '..', '..', 'knee_angle_gait.csv')


def on_connect(client, userdata, flags, rc, properties):
    print(f"Connected with result code {rc}")


def downsample(angles, n_samples):
    """Resample the angle curve to n_samples points, evenly spaced over the cycle."""
    x_old = np.linspace(0, 1, len(angles))
    x_new = np.linspace(0, 1, n_samples)
    return np.interp(x_new, x_old, angles).tolist()


def send_angles(csv_path=CSV_PATH, speed=1.0, loop=True, motor_control=True, n_samples=None,
                log_path=None):
    """Publish the angles in csv_path to the 'marker' topic, in order,
    and drive the knee motor to each angle via the PID controller.

    speed: playback speed multiplier (2.0 = twice as fast, 0.5 = half speed).
    loop: if True, repeat the gait cycle forever.
    motor_control: if True, drive the physical motor via MotorPIDController.
    n_samples: if set, downsample the angle curve to this many evenly-spaced points.
    log_path: if set, save (time, target_angle, current_angle) to this CSV on exit.
    """
    df = pd.read_csv(csv_path)
    angles = df['angle'].tolist()

    if n_samples is not None:
        angles = downsample(angles, n_samples)

    dt = 1.0 / (10 * speed)  # base rate of 10 Hz, scaled by speed

    client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2, 'gait_csv_sender')
    client.on_connect = on_connect
    client.connect(broker, port)
    client.loop_start()

    motor = MotorPIDController() if motor_control else None
    log = []
    t_start = time.time()

    print(f"Sending {len(angles)} angles at speed={speed} ({dt * 1000:.1f} ms/step)")

    try:
        while True:
            for angle in angles:
                print(angle)
                client.publish('marker', int(angle), qos=0)
                if motor is not None:
                    motor.target = angle
                if log_path is not None:
                    log.append((time.time() - t_start, angle,
                                 motor.current if motor is not None else None))
                time.sleep(dt)
            if not loop:
                break
    except KeyboardInterrupt:
        client.publish('marker', 0, qos=0)
        time.sleep(0.1)  # give the publish time to go out before disconnecting
    finally:
        if motor is not None:
            motor.stop()
        client.loop_stop()
        client.disconnect()
        if log_path is not None:
            pd.DataFrame(log, columns=['time', 'target_angle', 'current_angle']).to_csv(
                log_path, index=False)
            print(f"Saved {len(log)} target/current angle samples to {log_path}")


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--speed', type=float, default=2, help='Playback speed multiplier')
    parser.add_argument('--csv', type=str, default=CSV_PATH, help='Path to angle csv file')
    parser.add_argument('--no-loop', action='store_true', help='Send the cycle once and stop')
    parser.add_argument('--debug', action='store_true', help='Disable motor control')
    parser.add_argument('--n-samples', type=int, default=90,
                         help='Downsample the angle curve to N evenly-spaced samples')
    parser.add_argument('--log-csv', type=str, default=None,
                         help='Save target vs. current angle over time to this CSV')
    args = parser.parse_args()

    send_angles(csv_path=args.csv, speed=args.speed, loop=not args.no_loop,
                motor_control=not args.debug, n_samples=args.n_samples,
                log_path=args.log_csv)
