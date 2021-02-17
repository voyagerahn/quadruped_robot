from os import system, name

import numpy as np
import time

from src.Command import Command
from src.Config import Configuration
from src.Controller import Controller
from src.Kinematics import four_legs_inverse_kinematics
from src.multiprocess_kb import KeyInterrupt
from src.serial import ArduinoSerial
from src.State import State
from multiprocessing import Process

def consoleClear():
    # for windows
    if name == 'nt':
        _ = system('cls')

        # for mac and linux(here, os.name is 'posix')
    else:
        _ = system('clear')

def main(id,command_status):

    arduino = ArduinoSerial('COM4')  # need to specify the serial port

    # Create config
    config = Configuration()

    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()
    last_loop = time.time()
    command = Command()

    while True:
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # calculate robot step command from keyboard inputs
        result_dict = command_status.get()
        print(result_dict)
        command_status.put(result_dict)

        x_vel = result_dict['IDstepLength']
        y_vel = result_dict['IDstepWidth']
        command.yaw_rate = result_dict['IDstepAlpha']
        command.horizontal_velocity=np.array([x_vel, y_vel])

        #arduinoLoopTime, Xacc, Yacc, realRoll, realPitch = arduino.serialRecive()  # recive serial(IMU part)

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = ( np.array([1, 0, 0, 0])   )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        controller.run(state, command)


        arduino.serialSend(state.joint_angles[:,0],state.joint_angles[:,1],state.joint_angles[:,2],state.joint_angles[:,3])
        consoleClear()


if __name__ == '__main__':
    try:
        config = Configuration()

        # Keyboard input Process
        KeyInputs = KeyInterrupt()
        KeyProcess = Process(target=KeyInputs.keyInterrupt, args=(1, KeyInputs.key_status, KeyInputs.command_status))
        KeyProcess.start()

        # Main Process
        main(2,KeyInputs.command_status)

        print("terminate KeyBoard Input process")
        if KeyProcess.is_alive():
            KeyProcess.terminate()
    except Exception as e:
        print(e)
    finally:
        print("Done... :)")