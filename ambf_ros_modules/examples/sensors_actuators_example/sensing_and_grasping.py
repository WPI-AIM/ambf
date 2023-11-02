from ambf_client import Client
import rospy
import time
import sys
if sys.version_info[0] >= 3:
    from tkinter import *
else:
    from Tkinter import *


sensor_obj = None
actuators = []
actuators_activation_state = []
grasp = False


def grasp_button_cb():
    global grasp
    grasp = True
    print('GRASP REQUESTED')


def release_button_cb():
    global grasp
    grasp = False
    print('RELEASE REQUESTED')


def print_sensors_state(sensor_obj):
    print('Sensor Array Triggers: [', end=' ')
    for i in range(sensor_obj.get_count()):
        print(sensor_obj.is_triggered(i), end=' ')

    print(']')


def grasp_object(grasp):
    sensor_count = sensor_obj.get_count()

    if grasp:
        for i in range(sensor_count):
            if sensor_obj.is_triggered(i) and actuators_activation_state[i] is False:
                obj_name = sensor_obj.get_sensed_object(i)
                print('Grasping ', obj_name, ' via actuator ', i)
                actuators[i].actuate(obj_name)
                actuators_activation_state[i] = True
    else:
        for i in range(sensor_count):
            actuators[i].deactuate()
            if actuators_activation_state[i] is True:
                print('Releasing object from actuator ', i)
            actuators_activation_state[i] = False


def grasp_object_via_sensor_name(grasp):
    global sensor_obj
    sensor_count = sensor_obj.get_count()

    if grasp:
        for i in range(sensor_count):
            if sensor_obj.is_triggered(i) and actuators_activation_state[i] is False:
                actuators[i].actuate_from_sensor_data(sensor_obj.get_identifier())
                actuators_activation_state[i] = True
                print('Actuating actuator ', i, 'named: ', actuators[i].get_name())
    else:
        for i in range(sensor_count):
            actuators[i].deactuate()
            if actuators_activation_state[i] is True:
                print('Releasing object from actuator ', i)
            actuators_activation_state[i] = False


def main():
    global sensor_obj, actuators, actuators_activation_state, grasp
    c = Client('sensor_actuators_example')
    c.connect()
    # We have a sensor array (4 individual sensing elements)
    sensor_obj = c.get_obj_handle('tip_sensor_array')

    # We have four corresponding constraint actuators, that we are going to actuate based on the trigger events from the
    # above sensors. Note that there is not explicit relation between a sensor and an actuator, it is we who are
    # connection the two.
    actuator0 = c.get_obj_handle('tip_actuator0')
    actuator1 = c.get_obj_handle('tip_actuator1')
    actuator2 = c.get_obj_handle('tip_actuator2')
    actuator3 = c.get_obj_handle('tip_actuator3')

    actuators = [actuator0, actuator1, actuator2, actuator3]
    actuators_activation_state = [False, False, False, False]
    grasp = False

    time.sleep(1.0)

    tk = Tk()
    tk.title("Sensing and Grasping Example")
    tk.geometry("250x150")
    grasp_button = Button(
        tk, text="Grasp", command=grasp_button_cb, height=3, width=50, bg="red")
    release_button = Button(
        tk, text="Release", command=release_button_cb, height=3, width=50, bg="green")
    grasp_button.pack()
    release_button.pack()

    counter = 0
    while not rospy.is_shutdown():
        try:
            tk.update()
            if not (counter % 50):
                print_sensors_state(sensor_obj)
                counter = 0
            # grasp_object(grasp)
            grasp_object_via_sensor_name(grasp)
            time.sleep(0.02)
            counter = counter + 1
        except KeyboardInterrupt:
            print('Exiting Program')
            break


if __name__ == '__main__':
    main()
