import serial.tools.list_ports
import time
import pygame
import os
import pandas as pd
from timeit import default_timer as timer

os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
pygame.init()
screen = pygame.display.set_mode((50, 50))

port = "/dev/cu.usbmodem1101"
ser = serial.Serial(port, baudrate=115200, timeout=1)
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

    traj_rightarm = pd.read_csv("data/rightarm.csv")

    start_time = timer()
    while not done:
        
        elapsed = (timer() - start_time) * 1000

        handle_joysticks()

        (servos[0], servos[1]) = interp(traj_rightarm, elapsed, ['dxl_pos[0]', 'dxl_pos[1]'], speed=1) #maybe use period as parameter instead

        cmd_str = ""
        for i in range(6):
            cmd_str += f"s{i}:{int(servos[i])}\n"
        for i in range(2):
            cmd_str += f"m{i}:{int(motors[i])}\n"
        cmd_str += "\t"

        print(cmd_str)
        
        # if(len(joysticks.values()) > 0):
        #     print(joysticks.values()['0'])



        ser.write(cmd_str.encode())
        time.sleep(0.001)


def interp(traj_df, elapsed, labels, speed=1):
    period = traj_df['elapsed'][len(traj_df)-1]/speed

    phase = elapsed/period
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
    return interpolated
        


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