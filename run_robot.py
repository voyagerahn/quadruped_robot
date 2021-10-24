import numpy as np
import time
from src.IMU import IMU
from src.Controller import Controller
from src.JoystickInterface import JoystickInterface
from src.State import State, BehaviorState
from gongneungdynamics.Config import Configuration
from gongneungdynamics.Kinematics import four_legs_inverse_kinematics
from multiprocessing import Process
from Cam import takePicture
import time
from servo_controller import PWMInterface
import subprocess

def main(state, use_imu=False):
    """Main program
    """

    # Create config
    config = Configuration()
    hardware_interface = PWMInterface()

    # Create imu handle
    if use_imu:
        imu = IMU(port="/dev/ttyACM0")
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    print("Creating joystick listener...")
    joystick_interface = JoystickInterface(config)
    print("Done.")

    last_loop = time.time()

    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)
    # Wait until the activate button has been pressed
    capture_Process = Process(target=capture_img, args=(2,))
    last_time = time.time() - 15
    while True:
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

            """# Step the controller forward by dt
            controller.run(state, command)

            # Update the pwm widths going to the servos
            angle = state.joint_angles.T + np.deg2rad(90)
            hardware_interface.servoRotate(angle)
            svAngle = hardware_interface.getServoAngles()
            print(svAngle)"""

            # Mode capture image
            if(state.capture_state == True and time.time() - last_time >= 15):
                state.capture_state = False
                state.behavior_state = BehaviorState.REST
                last_state = state.last_state
                move_servo(state, command, controller, hardware_interface)
                state.behavior_state = last_state
                print("now -  last : ", time.time()-last_time)
                time.sleep(1)
                print("sleep")
                capture_Process.start()
                time.sleep(3)
                last_time = time.time()
            else : 
                state.behavior_state = state.last_state
                move_servo(state, command, controller, hardware_interface)

def move_servo(state, command, controller, hardware_interface):
    # Step the controller forward by dt
    controller.run(state, command)

    # Update the pwm widths going to the servos
    angle = state.joint_angles.T + np.deg2rad(90)
    hardware_interface.servoRotate(angle)
    svAngle = hardware_interface.getServoAngles()
    print(svAngle)

     
def capture_img(id):
    takePicture()

if __name__ =='__main__':
    try:
        state = State()
        #subprocess.call("gio mount -s gphoto2", shell=True)

        # main Process
        main(state)

    except Exception as e:
        print(e)
    finally:
        print("Done...:)")
