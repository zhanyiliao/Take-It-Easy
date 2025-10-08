from PIL import Image
from scipy.spatial.distance import euclidean
import json
import math
import numpy as np
import paho.mqtt.client as mqtt
import sys
import time
import threading
import traceback

from applications import *
from devices import *

# --------------------------------------------------

MQTT_HOST = '192.168.87.193'
SUBSCRIBER_TOPIC = 'takeiteasyMQTT/1'
PUBLISHER_TOPIC = 'takeiteasy'
SPEED = 255
STATE_SPACE = './env_lib_2.png'

CURRENT = (0.0, 0.0)
RUNNING = False
SHUTDOWN = False

# --------------------------------------------------

def main(speed, state_space, destination, arrival_range=20):

    global CURRENT

    pf = PathFinder(np.mean(Image.open(state_space), axis=2), step_length=20)
    pca = PCA(address=0x40, bus=1, write_delay=0.1)
    locator = Locator(None, port='/dev/ttyUSB0')
    locator.locate(clear=True)

    print('Let\'s go')
    is_arrived = False
    time.sleep(2)

    try:

        while not is_arrived and not SHUTDOWN:

            time.sleep(0.25)
            pca.brake(False)
            pca.turn(True, True)
            CURRENT = locator.locate()

            route = pf.find_route(CURRENT, destination)
            if route:
                relay = np.add(CURRENT, route[0])
                print('From %s -> %s' % (CURRENT, relay.tolist()))
            else:
                print('Can not find route')
                pca.move(speed, speed)
                pca.turn(False, False)
                while not pf.find_route(locator.locate(clear=True), destination):
                    time.sleep(2)
                continue

            true_degrees = math.degrees(np.arctan2(*(relay - CURRENT)[::-1]))
            degrees_error = angle_normalize(locator.get_angle() - true_degrees)

            if abs(degrees_error) > 179:

                # deg_m = mpu.get_angle()
                pca.move(speed // 2, speed // 2)

                if degrees_error > 0:
                    pca.turn(False, True) # too right
                else:
                    pca.turn(True, False) # too left

                time.sleep(abs(degrees_error) / 60)
                # while abs(mpu.get_angle() - deg_m) + 45 < abs(degrees_error):
                #     time.sleep(0.5)
                
                pca.move(speed, speed)
                pca.turn(True, True)
                locator.locate(clear=True)
                time.sleep(1.5)

            else:

                for _ in range(3):

                    CURRENT = locator.locate()

                    if euclidean(CURRENT, destination) < arrival_range or SHUTDOWN:
                        is_arrived = True
                        break

                    degrees_error = angle_normalize(locator.get_angle() - true_degrees)
                    margin = abs(math.ceil(degrees_error * speed / 64))
                    margin = min(margin, speed * 2)
                    
                    if degrees_error > 0:
                        s1, s4 = (speed - margin, speed) if speed > margin else (0, margin)
                    else:
                        s1, s4 = (speed, speed - margin) if speed > margin else (margin, 0)
                
                    pca.move(s1, s4)
                    print('%s <%3d %3d>' % (CURRENT, s1, s4))

    except Exception:
        print(traceback.format_exc())

    except KeyboardInterrupt:
        pass

    finally:
        CURRENT = locator.locate(clear=True)
        del pca

    print('End of route')

# --------------------------------------------------

if __name__ == '__main__':

    client = mqtt.Client()

    def on_connect(mqttc, obj, flags, rc):
        print('MQTT connected: ' + str(rc))

    def on_message(mqttc, obj, msg):

        data = json.loads(msg.payload.decode())
        print(data)
    
        def job():
            global RUNNING, SHUTDOWN
            if data['shutdown']:
                SHUTDOWN = True
            else:
                SHUTDOWN = False
                RUNNING = True
                main(SPEED, STATE_SPACE, (data['x'], data['y']))
                RUNNING = False

        t = threading.Thread(target=job)
        t.start()

    client.on_message = on_message
    client.on_connect = on_connect
    client.connect(MQTT_HOST, 1883, 60)

    def subscriber():
        client.subscribe(SUBSCRIBER_TOPIC, 0)
        client.loop_forever()

    def publisher():
        while True:
            client.publish(PUBLISHER_TOPIC, '%f %f %s' % (CURRENT[0], CURRENT[1], not RUNNING))
            time.sleep(0.25)

    try:
        thread_p = threading.Thread(target=publisher)
        thread_s = threading.Thread(target=subscriber)
        thread_p.start()
        thread_s.start()
        thread_p.join()
        thread_s.join()
    finally:
        SHUTDOWN = True
        print('End of program')
