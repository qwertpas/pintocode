import serial.tools.list_ports
import time
import pygame
import os
import pandas as pd
from timeit import default_timer as timer
from periodics import PeriodicSleeper
import numpy as np

os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
pygame.init()
screen = pygame.display.set_mode((50, 50))

port = "/dev/cu.usbmodem1101"
ser = None
# ports = serial.tools.list_ports.comports()

joysticks = {}
joy_data = {
    'leftx': 0,
    'lefty': 0,
    'rightx': 0,
    'righty': 0,
}

servos = [0, 1, 2, 3, 4, 5]
motors = [7, 8]

done = False
def main():
    global ser



    traj_rightarm = pd.read_csv("data/rightarm.csv")

    send_handler = PeriodicSleeper(send_to_estop, 0.01)

    start_time = timer()
    while not done:

        while(ser is None):
            try:
                ser = serial.Serial(port, baudrate=115200, timeout=1, stopbits=serial.STOPBITS_TWO)
            except:
                ser = None
                print("plug in estop pls")
                time.sleep(0.1)
        
        elapsed = (timer() - start_time) * 1000

        handle_joysticks()

        [servos[0], servos[1]] = interp(traj_rightarm, elapsed, ['dxl_pos[0]', 'dxl_pos[1]'], speed=2) #maybe use period as parameter instead
        [servos[3], servos[2]] = np.array([4095, 4095]) - interp(traj_rightarm, elapsed, ['dxl_pos[0]', 'dxl_pos[1]'], speed=2, phase_shift=0)

        
        time.sleep(0.01)

    send_handler.stop()


def send_to_estop():
    global ser
    if(ser is None):
        return

    cmd_str = ""
    for i in range(6):
        cmd_str += f"s{i}:{int(servos[i]):05}\n"
    for i in range(2):
        cmd_str += f"m{i}:{int(motors[i]):05}\n"
    cmd_str += "#\t"

    print(cmd_str)

    try:
        ser.write(cmd_str.encode())
    except:
        ser = None
        print("Estop disconnected")


def interp(traj_df, elapsed, labels, speed=1, phase_shift=0):
    period = traj_df['elapsed'][len(traj_df)-1]/speed

    phase = elapsed/period + phase_shift
    progress = (phase - int(phase)) * len(traj_df) #goes from 0 to length of traj
    index = int(progress)
    index_frac = progress - index

    if(index + 1 < len(traj_df)):
        index_next = index + 1
    else:
        index_next = 0

    interpolated = []
    for label in labels:
        interpolated.append((1-index_frac)*traj_df[label][index] + (index_frac)*traj_df[label][index_next])
    return np.array(interpolated)
        


def handle_joysticks():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            global done
            done = True

        if event.type == pygame.JOYDEVICEADDED:
            joy = pygame.joystick.Joystick(event.device_index)
            joysticks[joy.get_instance_id()] = joy
            print(f"Joystick {joy.get_instance_id()} connected")
        if event.type == pygame.JOYDEVICEREMOVED:
            del joysticks[event.instance_id]
            print(f"Joystick {event.instance_id} disconnected")

        for joystick in joysticks.values():
            joy_data['leftx'] = joystick.get_axis(0)
            joy_data['lefty'] = joystick.get_axis(1)
            joy_data['rightx'] = joystick.get_axis(2)
            joy_data['righty'] = joystick.get_axis(3)
            break #assume only one joystick


if(__name__ == "__main__"):
    main()