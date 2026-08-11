import os; os.system('clear')
import time
import pandas as pd
from paho.mqtt import client as mqtt_client
import leg.parameters as p


broker = p.BROKER_HOST
port = 1883
topic = "data"

CSV_PATH = os.path.join(os.path.dirname(__file__), '..', '..', 'knee_angle_gait.csv')


def on_connect(client, userdata, flags, rc, properties):
    print(f"Connected with result code {rc}")


def send_angles(csv_path=CSV_PATH, speed=1.0, loop=True):
    """Publish the angles in csv_path to the 'marker' topic, in order.

    speed: playback speed multiplier (2.0 = twice as fast, 0.5 = half speed).
    loop: if True, repeat the gait cycle forever.
    """
    df = pd.read_csv(csv_path)
    angles = df['angle'].tolist()

    dt = 1.0 / (10 * speed)  # base rate of 10 Hz, scaled by speed

    client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2, 'gait_csv_sender')
    client.on_connect = on_connect
    client.connect(broker, port)
    client.loop_start()

    print(f"Sending {len(angles)} angles at speed={speed} ({dt * 1000:.1f} ms/step)")

    try:
        while True:
            for angle in angles:
                client.publish('marker', int(angle), qos=0)
                time.sleep(dt)
            if not loop:
                break
    except KeyboardInterrupt:
        client.publish('marker', 0, qos=0)
        time.sleep(0.1)  # give the publish time to go out before disconnecting
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--speed', type=float, default=1.0, help='Playback speed multiplier')
    parser.add_argument('--csv', type=str, default=CSV_PATH, help='Path to angle csv file')
    parser.add_argument('--no-loop', action='store_true', help='Send the cycle once and stop')
    args = parser.parse_args()

    send_angles(csv_path=args.csv, speed=args.speed, loop=not args.no_loop)
