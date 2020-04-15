import pygame
import carla
import random
import time
import numpy as np
import cv2
import queue

# custom libraries
from ds2_controller import DS2_Controller # dualshock2 controller module
from lane_image_functions import * # lane detection image processing functions
from laneDetect import * # lane detection algorithm


""" Init Controller """
# init pygame
pygame.init()

# init joysticks functionality
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)  # get joystick
joystick.init()                         # init joystick
controller = DS2_Controller(joystick)


""" CONSTANTS """
IM_WIDTH    = 1280
IM_HEIGHT   = 720
GLOBAL_FONT = cv2.FONT_HERSHEY_SIMPLEX
CAR_SPAWN_POINT = carla.Transform(carla.Location(x=-30, y=207.5, z=1))



# set up video writer
out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (IM_WIDTH, IM_HEIGHT))

# list to store actor in the simulation
actor_list = []

# car control object
car_control = carla.VehicleControl()


""" FNS """
def get_image(image):
    i = np.array(image.raw_data)
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    res = i2[:, :, :3]

    out.write(res) # write video

    # # show video
    # cv2.imshow("", res)
    # cv2.waitKey(1)
    
    return res

def get_carla_status(vehicle_actor, image):
    # get carla status
    vehicle_tf    = vehicle_actor.get_transform()
    vehicle_loc   = [round(vehicle_tf.location.x, 2),
                    round(vehicle_tf.location.y, 2),
                    round(vehicle_tf.location.z, 2)]
    
    custom_org = (35, 50)
    custom_fontscale = 0.7
    custom_color = (255, 150, 120) # blue
    custom_thickness = 2

    image = cv2.UMat(image)
    res = cv2.putText(image, 
                        f"location:(x={vehicle_loc[0]},y={vehicle_loc[1]},z={vehicle_loc[2]})",
                        custom_org,
                        GLOBAL_FONT,
                        custom_fontscale,
                        custom_color,
                        custom_thickness)

    return res

def get_driving_status(vehicle_actor, image):
    # get driving status
    vehicle_tf    = vehicle_actor.get_transform()
    vehicle_angle =  round(vehicle_tf.rotation.yaw, 3) # degree
    drive_mode    = car_control.reverse
    hand_brake    = car_control.hand_brake

    acceleration  = vehicle_actor.get_acceleration() # m/s^2
    acceleration  = np.array([acceleration.x, acceleration.y, acceleration.z])
    absolute_acceleration = round(np.linalg.norm(acceleration, ord=2), 3)

    velocity      = vehicle_actor.get_velocity()
    velocity      = np.array([velocity.x, velocity.y, velocity.z])
    absolute_velocity = round(np.linalg.norm(velocity, ord=2), 3)
    
    
    # display
    custom_org = (1000, 600)
    custom_fontscale = 0.8
    custom_color = (50, 50, 200) # green
    custom_thickness = 2
    
    # image = cv2.putText(image, f"Driving Status", (930, 540), GLOBAL_FONT, custom_fontscale, custom_color, custom_thickness)

    # drive mode string
    drive_mode_str = "REV" if drive_mode else "FWD"
    image = cv2.putText(image, f"drive mode: {drive_mode_str}", (900, 570), GLOBAL_FONT, custom_fontscale, custom_color, custom_thickness)
    
    # brake
    hand_brake_str = "YES" if hand_brake else "NO"
    image = cv2.putText(image, f"brake: {hand_brake_str}", (900, 600), GLOBAL_FONT, custom_fontscale, custom_color, custom_thickness)

    # acceleration
    image = cv2.putText(image, f"acceleration: {absolute_acceleration} m/s^2", (900, 630), GLOBAL_FONT, custom_fontscale, custom_color, custom_thickness)

    # velocity
    image = cv2.putText(image, f"velocity: {absolute_velocity} m/s", (900, 660), GLOBAL_FONT, custom_fontscale, custom_color, custom_thickness)

    # angle
    res = cv2.putText(image, f"steering angle: {vehicle_angle} deg", (900, 690), GLOBAL_FONT, custom_fontscale, custom_color, custom_thickness)

    return res

def apply_car_control(control_obj):
    throttle_limit = 0.3
    steering_limit = 0.3

    # throttle and brake
    if controller.analog_left_y <= -0.2:
        control_obj.throttle = -controller.analog_left_y*throttle_limit
        control_obj.brake = 0
    elif controller.analog_left_y >= 0.2:
        control_obj.brake = controller.analog_left_y*throttle_limit
        control_obj.throttle = 0
    else:
        control_obj.throttle = 0

    # steering
    if controller.analog_right_x >= 0.1 or controller.analog_right_x <= -0.1:
        control_obj.steer = controller.analog_right_x*steering_limit
    else:
        control_obj.steer = 0

    # hand_brake
    if controller.L[0]:
        control_obj.hand_brake = True
    else:
        control_obj.hand_brake = False

    # reverse
    if controller.R[0]:
        control_obj.reverse = not control_obj.reverse

    # apply control to vehicle
    vehicle.apply_control(control_obj)

def spawn_camera(camera_blueprint, camera_spawnpoint):
    camera_blueprint.set_attribute("image_size_x", f"{IM_WIDTH}")
    camera_blueprint.set_attribute("image_size_y", f"{IM_HEIGHT}")
    camera_blueprint.set_attribute("fov", "80")
    camera_blueprint.set_attribute("lens_circle_falloff", "0")
    camera_blueprint.set_attribute("sensor_tick", "0.3")  # capture image every 0.3 seconds

    sensor = world.try_spawn_actor(camera_blueprint, camera_spawnpoint, attach_to=vehicle)

    return sensor

def show_image(image):
    cv2.imshow("", image)
    cv2.waitKey(1)



""" PROGRAM BEGINS """
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
    spawn_point = CAR_SPAWN_POINT  # our car spawnpoint

    # spawn a car
    vehicle = world.try_spawn_actor(car_bp, spawn_point)
    assert vehicle is not None, "Vehicle cannot be spawned!"
    print(f"Cybertruck spawned.")
    actor_list.append(vehicle)

    # get rgb camera blueprint and set it's attributes
    camera_bp = bp_lib.find("sensor.camera.rgb")

    # adjust sensor location relative to vehicle
    camera_spawn_point = carla.Transform(carla.Location(x=4.0, z=1.2), carla.Rotation(pitch=0.0))

    # spawn camera
    sensor = spawn_camera(camera_bp, camera_spawn_point)
    assert sensor is not None, "Camera cannot be spawned!"
    print(f"Camera spawned.")
    actor_list.append(sensor)

    # add image to queue when captured
    image_queue = queue.Queue()
    sensor.listen(image_queue.put)

    GameDone = False  # flag to stop this client (press Ctrl+C to stop)

    use_lane_detect = True

    """ GAME STARTS HERE """
    while not GameDone:
        world.tick()  # signal the server to update

        # read events when user does something
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                GameDone = True

        # update controller
        controller.update() # controller.debug()
        apply_car_control(car_control)

        # get image
        img = image_queue.get()
        img = get_image(img)
        
        # toggle lane detection
        use_lane_detect = not use_lane_detect if controller.R[1] else use_lane_detect
        if use_lane_detect:
            # lane detection
            warped, M, Minv = perspectiveTransform(img)
            bin_warped      = process_threshold(warped)
            left_fit, right_fit, slidingWin = laneWindowSearch(bin_warped)
            unwarped = cv2.warpPerspective(slidingWin, Minv, (IM_WIDTH, IM_HEIGHT))
            dist, side = getLaneOffset(unwarped, left_fit, right_fit)
            img = cv.addWeighted(img, 1, unwarped, 1, 0)

        # show general status
        img = get_carla_status(vehicle, img)
        img = get_driving_status(vehicle, img)

        # show image
        show_image(img)


finally:
    print(f"release video writer...")
    out.release()
    cv2.destroyAllWindows()

    print("Destroying actors...")
    for actor in actor_list:
        actor.destroy()
    print("done.")

