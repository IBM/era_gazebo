#!/usr/bin/env python

import carla
import random 
from time import sleep

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)

world = client.get_world()

actor_list = world.get_actors()

for ped in actor_list.filter('walker.*'):
	ped.destroy()
for veh in actor_list.filter('vehicle.carlamotors.carlacola'):
	veh.destroy()


# Add Truck

truck_bp = world.get_blueprint_library().find("vehicle.carlamotors.carlacola")

truck_spawn_point =  carla.Transform()
truck_spawn_point.location = carla.Location(-41.5, 40.0, 0.5)
truck_spawn_point.rotation = carla.Rotation(0,-90,0)

truck = world.spawn_actor(truck_bp, truck_spawn_point)


actor_list = world.get_actors()

# Print the location of all the speed limit signs in the world.
for traffic_light in actor_list.filter('traffic.traffic_light'):
#	print(traffic_light.id)
#	print(traffic_light.get_location())
	if (abs(traffic_light.get_location().x - (-31.636356))) < 0.1 : #Red light to stop the watcher
#		print(traffic_light.get_state())
		traffic_light.set_state(carla.TrafficLightState.Red)
		traffic_light.set_red_time(1000)
	if (abs(traffic_light.get_location().x - (-64.264191))) < 0.1 : #Red light to stop the watcher - opposite
#		print(traffic_light.get_state())
		traffic_light.set_state(carla.TrafficLightState.Red)
		traffic_light.set_red_time(1000)	
	if (abs(traffic_light.get_location().x - (-31.930841))) < 0.1 : #Green light for the moving car
#		print(traffic_light.get_state())
		traffic_light.set_state(carla.TrafficLightState.Green)
		traffic_light.set_green_time(1000)
	if (abs(traffic_light.get_location().x - (-62.352070))) < 0.1 : #Green light for the moving car - opposite
#		print(traffic_light.get_state())
		traffic_light.set_state(carla.TrafficLightState.Green)
		traffic_light.set_green_time(1000)	

# Add pedestrian

ped_blueprints = world.get_blueprint_library().filter("walker.*")

ped_spawn_point = carla.Transform()
ped_spawn_point.location = carla.Location(-36, 37, 0.7)

walker_bp = random.choice(ped_blueprints)
walker_bp.set_attribute('is_invincible', 'false')
player = world.spawn_actor(walker_bp, ped_spawn_point)
player_control = carla.WalkerControl()
player_control.speed = 5
pedestrian_heading=180
player_rotation = carla.Rotation(0,pedestrian_heading,0)
player_control.direction = player_rotation.get_forward_vector()


for toyota in actor_list.filter("vehicle.toyota.*"):
	print(toyota.get_location())

while toyota.get_location().distance(ped_spawn_point.location) > 20.0 :
	sleep(0.1)

#cmd = raw_input('Hit p to spawn a pedestrian\n')

#if cmd == "p" :

player.apply_control(player_control)

