import math
import time
import csv
from pymavlink import mavutil
import cv2
import numpy as np


def calculate_endpoint(start_x, start_y, angle, length, direction):

    end_x = start_x + length * math.cos(math.radians(angle)) * direction
    end_y = start_y + length * math.sin(math.radians(angle)) * direction

    return end_x, end_y

start_x =
start_y =
angle =
length =
direction =

end_x, end_y = calculate_endpoint(start_x, start_y, angle, length, direction)
print(f"Конечная точка: ({end_x}, {end_y})")



def connect_to_pixhawk():
    connection_string = 'udp:127.0.0.1:14550'
    mavutil.set_dialect('ardupilotmega')

    try:
        vehicle = mavutil.mavlink_connection(connection_string)
        return vehicle
    except Exception as e:
        print(f"We was lost him:{e}")
        return None


def write_coordinates_to_file(vehicle):
    with open('coordinates.txt', 'w') as f:
        writer = csv.writer(f, delimiter='\t')
        writer.writerow(['Day', 'Hour', 'Minute', 'Second', 'Latitude', 'Longitude'])

        start_time = time.time()
        while True:
            current_time = time.time() - start_time


            lat = vehicle.messages['GPS_RAW_INT'].lat
            lon = vehicle.messages['GPS_RAW_INT'].lon


            writer.writerow([
                int(current_time / (24 * 60 * 60)),  # Day
                int(current_time / 3600) % 24,  # Hour
                int(current_time / 60) % 60,  # Minute
                int(current_time) % 60,  # Second
                lat,
                lon
            ])




            time.sleep(1)
def checker (vehicle)
            roll = vehicle.messages['AHRS3'].roll
            pitch = vehicle.messages['AHRS3'].pitch
            if abs(roll) > 90 or abs(pitch) > 90:
                print("!!!Warning, risk of crashing your dump A**!!!")
                vehicle.commands.clear()
                vehicle.commands.upload()

                vehicle.armed = False
                vehicle.mode = mavutil.mavlink.MAV_MODE_MANUAL_DISABLED
                vehicle.close()

vehicle = connect_to_pixhawk()
if vehicle:
    write_coordinates_to_file(vehicle)
ideo_path = 'path/to/video.mp4'

def data():
    data_archive = np.load('path/to/data_archive.npy')
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print("Zzzz.... Zzzz..... Zzzzz...")
        exit()


    while True:

        ret, frame = cap.read()

        if not ret:
            break
        grayscale_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        process = cv2.some_processing_function(grayscale_frame, data_archive)

        cv2.imshow('Frame', process)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
