import pygame
import carla
import random
import time
import numpy as np
import cv2
import queue
from ds2_controller import DS2_Controller


""" Init Controller """
# init pygame
pygame.init()

# init joysticks functionality
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)  # get joystick
joystick.init()                         # init joystick
controller = DS2_Controller(joystick)

""" CONSTANTS """
IM_WIDTH = 1280
IM_HEIGHT = 720



# set up video writer
out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (IM_WIDTH, IM_HEIGHT))

# list to store actor in the simulation
actor_list = []
car_control = carla.VehicleControl()


""" FNS """
def process_img(image):
    i = np.array(image.raw_data)
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    res = i2[:, :, :3]

    out.write(res) # write video

    # show video
    cv2.imshow("", res)
    cv2.waitKey(1)
    
    # return res / 255.0


def apply_car_control():
    # throttle and brake
    if controller.analog_left_y <= -0.2:
        car_control.throttle = -controller.analog_left_y*0.3
        car_control.brake = 0
    elif controller.analog_left_y >= 0.2:
        car_control.brake = controller.analog_left_y*0.3
        car_control.throttle = 0
    else:
        car_control.throttle = 0

    # steering
    if controller.analog_right_x >= 0.1 or controller.analog_right_x <= -0.1:
        car_control.steer = controller.analog_right_x*0.3
    else:
        car_control.steer = 0

    # hand_brake
    if controller.L[0]:
        car_control.hand_brake = True
    else:
        car_control.hand_brake = False

    # reverse
    if controller.R[0]:
        car_control.reverse = not car_control.reverse

    # apply control to vehicle
    vehicle.apply_control(car_control)


def spawn_camera(camera_blueprint, camera_spawnpoint):
    camera_blueprint.set_attribute("image_size_x", f"{IM_WIDTH}")
    camera_blueprint.set_attribute("image_size_y", f"{IM_HEIGHT}")
    camera_blueprint.set_attribute("fov", "130")
    camera_blueprint.set_attribute("lens_circle_falloff", "0")
    camera_blueprint.set_attribute("sensor_tick", "0.2")  # capture image every 0.3 seconds

    sensor = world.try_spawn_actor(camera_blueprint, camera_spawnpoint, attach_to=vehicle)

    return sensor




try:
    # connect to carla server
    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)
    print(f"Connected to server.")

    # get world
    world = client.get_world()

    # load blueprints
    bp_lib = world.get_blueprint_library()

    # set weather
    weather = carla.WeatherParameters(
        cloudiness=0.0,
        precipitation=0.0,
        sun_altitude_angle=90.0)
    world.set_weather(weather)
    print(f"Weather set.")

    # get car blueprint
    car_bp = bp_lib.find("vehicle.tesla.cybertruck")
    spawn_point = carla.Transform(carla.Location(x=-30, y=207.5, z=1))  # our car spawnpoint

    # spawn a car
    vehicle = world.try_spawn_actor(car_bp, spawn_point)
    assert vehicle is not None, "Vehicle cannot be spawned!"
    print(f"Cybertruck spawned.")
    actor_list.append(vehicle)

    # get rgb camera blueprint and set it's attributes
    camera_bp = bp_lib.find("sensor.camera.rgb")

    # adjust sensor location relative to vehicle
    camera_spawn_point = carla.Transform(carla.Location(x=4.0, z=1.7), carla.Rotation(pitch=-18.0))

    # spawn camera
    sensor = spawn_camera(camera_bp, camera_spawn_point)
    assert sensor is not None, "Camera cannot be spawned!"
    print(f"Camera spawned.")
    actor_list.append(sensor)

    # add image to queue when captured
    image_queue = queue.Queue()
    sensor.listen(image_queue.put)

    GameDone = False  # flag to stop this client (press Ctrl+C to stop)

    # GAME HERE
    while not GameDone:
        world.tick()  # signal the server to update

        # read events when user do something
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                GameDone = True

        # update controller
        controller.update() # controller.debug()
        apply_car_control()

        # get image and process
        img = image_queue.get()
        process_img(img)


finally:
    print(f"release video writer...")
    out.release()
    cv2.destroyAllWindows()

    print("Destroying actors...")
    for actor in actor_list:
        actor.destroy()
    print("done.")




""" UNUSED CODE FRAGMENTS """
def print_loc(obj):
    obj = vehicle.get_location()
    print(f"x: {obj.x}, y: {obj.y}, z: {obj.z}")