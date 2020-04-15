import pygame
import carla
import random
import time
import numpy as np
import cv2


pygame.init()

# list to store actor in the simulation
actor_list = []

# this contain the spawn points of vehicle and walker
walker_spawn_points = [
    (-17, 207, 3), # lane 1
    (-8, 200, 3),  # lane 2
    (-1, 195, 3)   # lane 3
]

vehicle_spawn_points = [
    (5, 207, 3),   # lane 1, rightmost (x=along the road, y=change lane, z=height)
    (30, 204, 3),   # lane 2
    (40, 201, 3),   # lane 3
    (60, 201, 3),
    (70, 207, 3),
    (90, 207, 3)
]

try:
    # connect to carla server
    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)
    print(f"Connected to server.")

    # get world
    world = client.get_world()

    # load all blueprints
    bp_lib = world.get_blueprint_library()

    # load obstacle blueprints
    walker_bp = bp_lib.filter("pedestrian")
    vehicle_bp = bp_lib.filter("vehicle")


    def spawn_obstacle(blueprints, spawnpoints, num):
        for i in range(num):
            spawn_point = carla.Transform(carla.Location(x=spawnpoints[i][0],
                                                         y=spawnpoints[i][1],
                                                         z=spawnpoints[i][2]))
            
            entity = world.try_spawn_actor(blueprints[i], spawn_point)
            assert entity is not None, "Entity can't be spawned!"
            print(f"Entity Spawned")
            actor_list.append(entity)
    
    # spawn_obstacle(walker_bp, walker_spawn_points, 3)
    spawn_obstacle(vehicle_bp, vehicle_spawn_points, 6)

        

    GameDone = False  # flag to stop this client (press Ctrl+C to stop)

    # GAME HERE
    while not GameDone:
        # world.tick()  # signal the server to update

        # read events when user do something
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                GameDone = True



finally:
    print("Destroying actors...")
    for actor in actor_list:
        actor.destroy()
    print("done.")