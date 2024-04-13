import math
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
    'lefttrigger': 0,
    'righttrigger': 0,
    'A': 0,
    'B': 0,
    'X': 0,
    'Y': 0,
    '-': 0,
    'home': 0,
    '+': 0,
    'leftstickbutton': 0,
    'rightstickbutton': 0,
    'leftbumper': 0,
    'rightbumper': 0,
    'dpadup': 0,
    'dpaddown': 0,
    'dpadleft': 0,
    'dpadright': 0,
    'circle': 0,
}
axes_calibrated_dict = {
    'leftx+': False,
    'leftx-': False,
    'lefty+': False,
    'lefty-': False,
    'rightx+': False,
    'rightx-': False,
    'righty+': False,
    'righty-': False
}

global axes_calibrated
axes_calibrated = False

message = ''
messagebuffer = ''
telemetry = {
    'vbus': 0
} #data back

servos = [2048] * 5
motor_pwrs = [0]*2
motor_pos = [0]*2

task = 'idle'
task_starttime = 0

aux = 0

low_battery = False

done = False
def main():
    global ser
    global servos, motor_pwrs, motor_pos, aux
    global task, task_starttime
    global telemetry, low_battery

    traj_rightarm = pd.read_csv("data/backandforth.csv")

    servos = [1012, 1216, 2815, 3008, 3900]
    motor_pwrs = [0,0]

    send_handler = PeriodicSleeper(send_to_estop, 0.01)
    

    start_time = timer()
    while not done:
        elapsed = (timer() - start_time) * 1000 #milliseconds
        task_elapsed = elapsed - task_starttime

        while(ser is None):
            try:
                ser = serial.Serial(port, baudrate=115200, timeout=1, stopbits=serial.STOPBITS_TWO)
            except:
                ser = None
                print("plug in estop pls")
                time.sleep(0.1)
        
        handle_joysticks()
        recv_from_estop()

        poslib = [
            { #top motor A
                # 'extend': 25,
                'extend': -249,
                # 'retract': 270, #actually -37 but might overshoot and overwrap
                'retract': 13, #actually -37 but might overshoot and overwrap
            },
            { #bottom motor B
                'extend': 273,
                'retract': 160,
            }
        ]

        def set_motor_pos(motor_id, pos_proportion, duty):
            motor_pwrs[motor_id] = duty
            start = poslib[motor_id]['retract']
            end = poslib[motor_id]['extend']
            motor_pos[motor_id] = start + pos_proportion*(end-start)

        if(task == 'idle'):
            task_starttime = elapsed
            if(joy_data['B']): #arm pose to flip over
                # if(joy_data['dpadleft']):
                #     task = 'recoverleft'
                # elif(joy_data['dpadright']):
                #     task = 'recoverright'
                # servos = [2417, 2465, 1730, 1717, 2048]

                # servos[0] = 2417 #lower
                # servos[1] = 2465
                # servos[2] = 1730
                # servos[3] = 1717

                servos[0] = 3105 #all the way up
                servos[1] = 3472
                servos[2] = 601
                servos[3] = 960
            elif(joy_data['A']):
                servos[0] = 1369
                servos[1] = 459
                servos[2] = 3614
                servos[3] = 2591
            elif(joy_data['dpaddown']):
                task = 'sit'
            elif(joy_data['dpadleft']):
                task = 'turnleft'
            elif(joy_data['dpadright']):
                task = 'turnright'
            elif(joy_data['dpadup']):
                if(joy_data['leftbumper']):
                    task = 'longjump'
                else:
                    task = 'bound'
            elif(joy_data['+']):
                task = 'boogie'
            else: #voltage control
                motor_pwrs[0] = joy_data['lefty']*150
                motor_pwrs[1] = joy_data['righty']*150
                motor_pos[0] = math.copysign(999, joy_data['lefty'])
                motor_pos[1] = math.copysign(999, joy_data['righty'])
                motor_pos[0] = clip(math.copysign(999, joy_data['lefty']), poslib[0]['extend'], poslib[0]['retract'])
                motor_pos[1] = clip(math.copysign(999, -joy_data['righty']), poslib[1]['extend'], poslib[1]['retract'])
                # motor_pos[1] = math.copysign(999, joy_data['righty'])


        

        if(task == 'sit'):
            if(task_elapsed < 300):
                servos = [1012, 1216, 2815, 3008, 2471]
            elif(task_elapsed < 600):
                servos = [1586, 503, 3523, 2381, 2471]
            else:
                task = 'idle'

        if(task == 'boogie'):
            speed = 1.5
            if(task_elapsed < 100):
                [servos[0], servos[1]] = interp(traj_rightarm, elapsed, ['dxl_pos[0]', 'dxl_pos[1]'], speed=speed) #maybe use period as parameter instead
                [servos[3], servos[2]] = np.array([4095, 4095]) - interp(traj_rightarm, elapsed, ['dxl_pos[0]', 'dxl_pos[1]'], speed=speed, phase_shift=0.5)
                servos[4] = 3072
                # motor_pos[1] = poslib[1]['extend']
                # motor_pwrs[1] = 150
            if(task_elapsed < 200):
                [servos[0], servos[1]] = interp(traj_rightarm, elapsed, ['dxl_pos[0]', 'dxl_pos[1]'], speed=speed) #maybe use period as parameter instead
                [servos[3], servos[2]] = np.array([4095, 4095]) - interp(traj_rightarm, elapsed, ['dxl_pos[0]', 'dxl_pos[1]'], speed=speed, phase_shift=0.5)
                servos[4] = 3072
                # motor_pos[1] = poslib[1]['retract']
                # motor_pwrs[1] = 150
            else:
                task = 'idle'

        # if(task == 'bound'):
        #     if(task_elapsed < 30):
        #         # motor_pos[0] = poslib[0]['extend']
        #         # motor_pwrs[0] = 0
        #         servos = [1586, 503, 3523, 2381, 2471] #up
        #         # servos = [2742, 2053, 1987, 990, 3072] #out
        #         # motor_pos[1] = poslib[1]['extend']
        #         # motor_pwrs[1] = 100

        #     elif(task_elapsed < 80):
        #         servos = [2742, 2053, 1987, 990, 3072] #out
        #     elif(task_elapsed < 260):
        #         motor_pos[0] = poslib[0]['extend']
        #         motor_pwrs[0] = 150
        #         motor_pos[1] = poslib[1]['extend']
        #         motor_pwrs[1] = 150
        #         # servos = [1915, 1644, 2480, 2208, 2471]
        #     elif(task_elapsed < 500):
        #         motor_pos[0] = poslib[0]['retract']
        #         motor_pwrs[0] = 150
        #         motor_pos[1] = poslib[1]['retract']
        #         motor_pwrs[1] = 150
        #         servos = [1915, 1644, 2480, 2208, 2471]
                
        #     elif(task_elapsed < 550):
        #         servos = [1012, 1216, 2815, 3008, 2471]
        #     else:
        #         task = 'idle'

        if(task == 'bound'):
            power = 150
            if(task_elapsed < 50):
                servos = [1586, 503, 3523, 2381, 3300] #up
            elif(task_elapsed < 100):
                servos = [2968, 2459, 1676, 969, 3072] #out
            elif(task_elapsed < 350):
                motor_pos[0] = poslib[0]['extend']
                motor_pwrs[0] = power
                motor_pos[1] = poslib[1]['extend']
                motor_pwrs[1] = power
            elif(task_elapsed < 550):
                motor_pos[0] = poslib[0]['retract']
                motor_pwrs[0] = power
                motor_pos[1] = poslib[1]['retract']
                motor_pwrs[1] = power
                servos = [1915, 1644, 2480, 2208, 3300]
            elif(task_elapsed < 650):
                servos = [1012, 1216, 2815, 3008, 3900] #home retracted
                for stick in joysticks.values():
                    stick.rumble(0.7, 0.0, 100)
            else:
                task = 'idle'

        if(task == 'longjump'):
            if(task_elapsed < 30):
                servos = [1586, 503, 3523, 2381, 3900] #up
            elif(task_elapsed < 80):
                servos = [2742, 2053, 1987, 990, 3072] #out
            elif(task_elapsed < 300):
                motor_pos[0] = poslib[0]['extend']
                motor_pwrs[0] = 250
                motor_pos[1] = poslib[1]['extend']
                motor_pwrs[1] = 250
            elif(task_elapsed < 550):
                motor_pos[0] = poslib[0]['retract']
                motor_pwrs[0] = 250
                motor_pos[1] = poslib[1]['retract']
                motor_pwrs[1] = 250
                servos = [1915, 1644, 2480, 2208, 3900]
            elif(task_elapsed < 750):
                servos = [1012, 1216, 2815, 3008, 3900] #home retracted
                for stick in joysticks.values():
                    stick.rumble(0.7, 0.0, 100)
            else:
                task = 'idle'

        if(task == 'turnleft'):
            speed = 1.2
            if(task_elapsed < 100/speed):
                # servos = [2048-250, 2048-500, 2818, 3010, 2336] #right arm up 
                servos = [1012, 1216, 2300, 2200, 3900] #left arm up, unexpand
                # set_motor_pos(1, 0.9, 100)
            elif(task_elapsed < 300/speed):
                servos = [1650, 850, 2300, 2200, 3072] #expand
                # set_motor_pos(1, 0.9, 100)
            elif(task_elapsed < 400/speed):
                servos = [1650, 850, 2818, 3010, 3072] #right arm down, left arm up, still expanded
                # set_motor_pos(1, 0.9, 100)
            elif(task_elapsed < 500/speed):
                servos = [1800, 1550, 2818, 3010, 3900] #right arm down, left arm up, unexpand
                # set_motor_pos(1, 0.9, 100)
            elif(task_elapsed < 600/speed):
                servos = [1012, 1216, 2815, 3008, 3900] #home retracted
                # set_motor_pos(1, 0.9, 100)
            else:
                task = 'idle'

        if(task == 'turnright'):
            speed = 1.2
            if(task_elapsed < 100/speed):
                servos = [1800, 1600, 2818, 3010, 3900] #right arm up 
            elif(task_elapsed < 300/speed):
                servos = [1800, 1600, 3200, 2300, 3072] #expand
            elif(task_elapsed < 400/speed):
                servos = [1012, 1216, 2818, 3010, 3072] #right arm down, left arm up, still expanded
            elif(task_elapsed < 500/speed):
                servos = [1012, 1216, 2300, 2200, 3900] #right arm down, left arm up, unexpand
            elif(task_elapsed < 600/speed):
                servos = [1012, 1216, 2815, 3008, 3900] #home retracted
            else:
                task = 'idle'

        # if(task == 'turnright'):
        #     if(task_elapsed < 600):
        #         [servos[0], servos[1]] = interp(traj_rightarm, task_elapsed, ['dxl_pos[0]', 'dxl_pos[1]'], speed=2) #maybe use period as parameter instead
        #         [servos[3], servos[2]] = np.array([4095, 4095]) - interp(traj_rightarm, task_elapsed, ['dxl_pos[0]', 'dxl_pos[1]'], speed=2, phase_shift=0.5)
        #         servos[4] = 2048 + 1024*np.sin(task_elapsed / 150) + 512
        #     elif(task_elapsed < 700):
        #         servos = [1012, 1216, 2815, 3008, 2700]
        #     else:
        #         task = 'idle'

        # if(task == 'turnleft'):
        #     if(task_elapsed < 600):
        #         [servos[0], servos[1]] = interp(traj_rightarm, task_elapsed, ['dxl_pos[0]', 'dxl_pos[1]'], speed=2) #maybe use period as parameter instead
        #         [servos[3], servos[2]] = np.array([4095, 4095]) - interp(traj_rightarm, task_elapsed, ['dxl_pos[0]', 'dxl_pos[1]'], speed=2, phase_shift=0.5)
        #         servos[4] = 2048 - 1024*np.sin(task_elapsed / 150) + 512
        #     elif(task_elapsed < 700):
        #         servos = [1012, 1216, 2815, 3008, 2700]
        #     else:
        #         task = 'idle'
                

        if(joy_data['X']):
            servos[4] = 3072
        if(joy_data['Y']):
            servos[4] = 2048

        
        for i in range(5):
            servos[i] = clip(int(servos[i]), 0, 4095)

        #position control
        # if(joy_data['dpaddown']):
        #     motor_pos[0] = 270
        #     motor_pwrs[0] = 100;
        # elif(joy_data['dpadup']):
        #     motor_pos[0] = 25
        #     motor_pwrs[0] = 100;
        # else:
        #     motor_pwrs[0] = joy_data['lefty']*200
        #     motor_pwrs[1] = joy_data['righty']*200
        #     motor_pos[0] = math.copysign(999, joy_data['lefty'])
        #     motor_pos[1] = math.copysign(999, joy_data['righty'])

        # motor_pos[0] += joy_data['lefty'] * 2
        # motor_pos[0] = min(max(motor_pos[0], min_motor_pos[0]), max_motor_pos[0])

        # motor_pos[1] += joy_data['righty'] * 1
        # motor_pos[1] = min(max(motor_pos[1], min_motor_pos[1]), max_motor_pos[1])

        # motor_pwrs[1] = 50;


        

        low_battery_threshold = 10.5
        if((elapsed > 1000 and telemetry['motor_power_on']==1 and telemetry['vbus'] > 0 and telemetry['vbus'] < low_battery_threshold) or low_battery):
            print("LOW BATTERY")
            low_battery = True
            for i in range(5):
                servos[i] = 2048
            for i in range(2):
                motor_pwrs[i] = 0
                

        aux = 0
        if(joy_data['circle'] and joy_data['home']):
            aux |= (1 << 0) #set bit 0 for toggle motor power
        if(joy_data['circle']):
            aux |= (1 << 1) #set bit 1 for enable servo torque


        
        time.sleep(0.01)

    send_handler.stop()



# print message cleanly to terminal, along with other station data
def print_message(message):
    global servos, motor_pwrs, motor_pos, aux
    if(len(joysticks) == 0):
        print("gamepad disconnected")
    else:
        if(not axes_calibrated):
            print("please calibrate joysticks")
            print("move axes in a circle")
        else:
            joy_df = pd.DataFrame([joy_data]).round(2)
            print(joy_df[['leftx', 'lefty', 'rightx', 'righty']].to_string(index=False))

            print('s:', np.int_(servos))
            print('mw:', np.int_(motor_pwrs))
            print('mo:', np.int_(motor_pos))
            # print('a:', format(aux, '05b'))
            print(f" a:{aux:05b}")
    print(f"aux: {aux}")
    print(f"task: {task}")
    print(message)
    print("_—————____—————____—————____—————\t")




def send_to_estop():
    global ser
    if(ser is None):
        return

    cmd_str = ""
    for i in range(5):
        cmd_str += f"s{i}:{servos[i]:05}\n"
    for i in range(2):
        cmd_str += f"mw{i}:{motor_pwrs[i]:05}\n"
    for i in range(2):
        cmd_str += f"mo{i}:{motor_pos[i]:05}\n"
    cmd_str += f"a:{aux:05b}"
    cmd_str += "#\t"

    # print(cmd_str)

    try:
        ser.write(cmd_str.encode())
    except Exception as e:
        print(e)
        ser = None
        print("Estop disconnected")


delimiter = '\t'
def recv_from_estop():
    # print(ser.read_all().decode("utf-8", errors='ignore'), end=None)
    global ser
    global messagebuffer, telemetry
    global message
    global axes_calibrated

    messagecount = 0

    try:
        if ser.in_waiting > 0:
            uarttext = ser.read_all().decode('utf-8', errors='ignore')
            
            ending=0
            while(uarttext):
                ending = uarttext.find(delimiter)
                if(ending == -1):
                    break

                message += uarttext[0:ending]

                print_message(message)
                messagebuffer = message
                # print(messagebuffer)

                lines = messagebuffer.split('\n')
                for line in lines:
                    parts = line.split(':')
                    key = parts[0].strip()
                    try:
                        value = float(parts[1].strip())
                    except Exception:
                        continue
                    telemetry[key] = value

                messagecount += 1
                message = "" #clear message
                uarttext = uarttext[ending+len(delimiter):] #front of buffer used up

            message = uarttext #whatver is left over
    except Exception as e:
        print(e)
        return


def interp(traj_df, elapsed, labels, speed=1, phase_shift=0, dir=1):
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
    global axes_calibrated
    global axes_calibrated_dict

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            global done
            done = True

        if event.type == pygame.JOYDEVICEADDED:
            joy = pygame.joystick.Joystick(event.device_index)
            joysticks[joy.get_instance_id()] = joy
            print(f"Joystick {joy.get_instance_id()} connected")
            axes_calibrated = False

        if event.type == pygame.JOYDEVICEREMOVED:
            del joysticks[event.instance_id]
            print(f"Joystick {event.instance_id} disconnected")

        for joystick in joysticks.values():
            joy_data['leftx'] = joystick.get_axis(0) 
            joy_data['lefty'] = -joystick.get_axis(1)
            joy_data['rightx'] = joystick.get_axis(2)
            joy_data['righty'] = -joystick.get_axis(3) # -1 to 1
            joy_data['lefttrigger'] = joystick.get_axis(4) # -1 to 1
            joy_data['righttrigger'] = joystick.get_axis(5) # -1 to 1
            joy_data['A'] = joystick.get_button(0)
            joy_data['B'] = joystick.get_button(1)
            joy_data['X'] = joystick.get_button(2)
            joy_data['Y'] = joystick.get_button(3)
            joy_data['-'] = joystick.get_button(4)
            joy_data['home'] = joystick.get_button(5) #make sure to disable Launchpad https://apple.stackexchange.com/questions/458669/how-do-i-disable-the-home-button-on-a-game-controller-in-macos
            joy_data['+'] = joystick.get_button(6)
            joy_data['leftstickbutton'] = joystick.get_button(7)
            joy_data['rightstickbutton'] = joystick.get_button(8)
            joy_data['leftbumper'] = joystick.get_button(9)
            joy_data['rightbumper'] = joystick.get_button(10)
            joy_data['dpadup'] = joystick.get_button(11)
            joy_data['dpaddown'] = joystick.get_button(12)
            joy_data['dpadleft'] = joystick.get_button(13)
            joy_data['dpadright'] = joystick.get_button(14)
            joy_data['circle'] = joystick.get_button(15)

            for axis_name in ['leftx', 'lefty', 'rightx', 'righty']: #deadband
                if(abs(joy_data[axis_name]) < 0.02):
                    joy_data[axis_name] = 0


            #seems like pygame needs to know the max axis value so need to move stick to max/min
            for axis_name in ['leftx', 'lefty', 'rightx', 'righty']:
                if(joy_data[axis_name] > 0.99):
                    axes_calibrated_dict[f"{axis_name}+"] = True
                elif(joy_data[axis_name] < -0.99):
                    axes_calibrated_dict[f"{axis_name}-"] = True
            
            #check if each axis calibrated, complete only all done and at home
            if(not axes_calibrated):
                axes_calibrated_try = True
                for calibration_label in axes_calibrated_dict:
                    if(axes_calibrated_dict[calibration_label] == False):
                        axes_calibrated_try = False
                        break
                for axis_name in ['leftx', 'lefty', 'rightx', 'righty']:
                    if(abs(joy_data[axis_name]) > 0.01):
                        axes_calibrated_try = False
                        break
                axes_calibrated = axes_calibrated_try
                if(axes_calibrated):
                    joystick.rumble(0.7, 0.0, 100) #indicate calibration done
                else:
                    for axis_name in ['leftx', 'lefty', 'rightx', 'righty']:
                        joy_data[axis_name] = 0

            break #assume only one joystick


def clip(x, _min, _max):
    if(_min > _max): #swap if min and max are reversed
        temp = _min
        _min = _max
        _max = temp
    if(x < _min):
        return _min
    elif(x > _max):
        return _max
    else:
        return x

if(__name__ == "__main__"):
    main()