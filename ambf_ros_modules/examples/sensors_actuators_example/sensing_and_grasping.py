from ambf_client import Client
import time
import Tkinter

global sensor_array, actuator_array, sensor_triggered, actuator_activated, grasp


def grasp_button_cb():
    global grasp
    grasp = True
    print('GRASP REQUESTED')


def release_button_cb():
    global grasp
    grasp = False
    print('RELEASE REQUESTED')


def grasp_object(grasp):
    global sensor_array, actuator_array, sensor_triggered, actuator_activated

    sensor_count = sensor_array.get_count()

    for i in range(sensor_count):
        if sensor_array.is_triggered(i):
            sensor_triggered[i] = True
        else:
            sensor_triggered[i] = False

    print('Sensor Array Triggers: ', sensor_triggered)

    if grasp:
        for i in range(sensor_count):
            if sensor_triggered[i] and actuator_activated[i] is False:
                obj_name = sensor_array.get_sensed_object(i)
                print('Grasping ', obj_name, ' via actuator ', i)
                actuator_array[i].actuate(obj_name)
                actuator_activated[i] = True
    else:
        for i in range(sensor_count):
            actuator_array[i].deactuate()
            if actuator_activated[i] is True:
                print('Releasing object from actuator ', i)
            actuator_activated[i] = False


def main():
    global sensor_array, actuator_array, sensor_triggered, actuator_activated, grasp
    c = Client('sensor_actuators_example')
    c.connect()
    # We have a sensor array (4 individual sensing elements)
    sensor_array = c.get_obj_handle('tip_sensor_array')

    # We have four corresponding constraint actuators, that we are going to actuate based on the trigger events from the
    # above sensors. Note that there is not explicit relation between a sensor and an actuator, it is we who are
    # connection the two.
    actuator0 = c.get_obj_handle('tip_actuator0')
    actuator1 = c.get_obj_handle('tip_actuator1')
    actuator2 = c.get_obj_handle('tip_actuator2')
    actuator3 = c.get_obj_handle('tip_actuator3')

    actuator_array = [actuator0, actuator1, actuator2, actuator3]
    sensor_triggered = [False, False, False, False]
    actuator_activated = [False, False, False, False]
    grasp = False

    time.sleep(1.0)

    tk = Tkinter.Tk()
    tk.title("Attache Needle")
    tk.geometry("250x150")
    grasp_button = Tkinter.Button(tk, text="Grasp", command=grasp_button_cb, height=3, width=50, bg="red")
    release_button = Tkinter.Button(tk, text="Release", command=release_button_cb, height=3, width=50, bg="green")
    grasp_button.pack()
    release_button.pack()

    while True:
        try:
            tk.update()
            grasp_object(grasp)
            time.sleep(0.02)
        except KeyboardInterrupt:
            print('Exiting Program')
            break


if __name__ == '__main__':
    main()
