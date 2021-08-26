from os import system, name

import numpy as np
import time

from src.Command import Command
from src.Config import Configuration
from src.Controller import Controller
from src.Kinematics import four_legs_inverse_kinematics
#from src.JoystickInterface import JoystickInterface
from src.multiprocess_kb import KeyInterrupt
from src.serial import ArduinoSerial
from src.State import State
from multiprocessing import Process

# from src.HardwareInterface import HardwareInterface

#angle_offset = np.array([[0 ,0 ,0 ,0] ,[0, 0, 0 ,0] ,[69 ,69 ,92 ,92]])

def consoleClear():
    # for windows
    if name == 'nt':
        _ = system('cls')

        # for mac and linux(here, os.name is 'posix')
    else:
        _ = system('clear')

def main(id ,command_status):

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

    # if using joystick
    """
    print("Creating joystick listener...")
    joystick_interface = JoystickInterface(config)
    print("Done.")
   
    while True:`
        print("Waiting for L1 to activate robot.")
        while True:
            command = joystick_interface.get_command(state)
            joystick_interface.set_color(config.ps4_deactivated_color)
            if command.activate_event == 1:
                break
            time.sleep(0.1)
        print("Robot activated.")
        joystick_interface.set_color(config.ps4_color)

        while True:
            now = time.time()
            if now - last_loop < config.dt:
                continue
            last_loop = time.time()

            # Parse the udp joystick commands and then update the robot controller's parameters
            command = joystick_interface.get_command(state)
            if command.activate_event == 1:
                print("Deactivating Robot")
                break

            # Read imu data. Orientation will be None if no data was available
            quat_orientation = (
                imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
            )
            state.quat_orientation = quat_orientation

            # Step the controller forward by dt
            controller.run(state, command)
            
            deg_angle =np.rad2deg(state.joint_angles) + angle_offset  # make angle rad to deg
            arduino.serialSend(deg_angle[: ,0] ,deg_angle[: ,1] ,deg_angle[: ,2] ,deg_angle[: ,3])

    """

    while True:
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # calculate robot step command from keyboard inputs
        result_dict = command_status.get()
        # print(result_dict)
        command_status.put(result_dict)

        x_vel = result_dict['IDstepLength']
        y_vel = result_dict['IDstepWidth']
        command.yaw_rate = result_dict['IDstepAlpha']
        command.horizontal_velocity = np.array([x_vel, y_vel])

        # recive serial(IMU part)
        #Xacc , Yacc , realRoll , realPitch = arduino.serialRecive()
        #print(Xacc,Yacc,realRoll,realPitch)
        #print(Yacc)

        # Read imu data. Orientation will be None if no data was available

        #quat_orientation = np.array([ 1,realRoll,realPitch,0]) # 1 realRoll realPitch 0
        quat_orientation = np.array([ 1,0,0,0]) # 1 realRoll realPitch 0

        #print(quat_orientation)
        #print(Xacc)
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        controller.run(state, command)

        deg_angle =np.rad2deg(state.joint_angles)# + angle_offset  # make angle rad to deg
        print(deg_angle)


        arduino.serialSend(deg_angle[: ,0] ,deg_angle[: ,1] ,deg_angle[: ,2] ,deg_angle[: ,3])
        consoleClear()


if __name__ == '__main__':
    try:
        config = Configuration()

        # Keyboard input Process
        KeyInputs = KeyInterrupt()
        KeyProcess = Process(target=KeyInputs.keyInterrupt, args=(1, KeyInputs.key_status, KeyInputs.command_status))
        KeyProcess.start()

        # Main Process
        main(2 ,KeyInputs.command_status)

        print("terminate KeyBoard Input process")
        if KeyProcess.is_alive():
            KeyProcess.terminate()
    except Exception as e:
        print(e)
    finally:
        print("Done... :)")


