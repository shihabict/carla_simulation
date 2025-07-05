import carla, time

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.load_world('Town05')

bp_lib = world.get_blueprint_library()
spawn_points = world.get_map().get_spawn_points()

# choose a spawn point and nudge it up slightly
start_point = spawn_points[10]

leader_bp = bp_lib.find('vehicle.tesla.model3')

# try_spawn_actor returns None on failure
leader = world.try_spawn_actor(leader_bp, start_point)

# move simulator view to the car
spectator = world.get_spectator()
start_point.location.z = start_point.location.z+3 #start_point was used to spawn the car but we move 1m up to avoid being on the floor
start_point.Rotation.yaw = 100
print(start_point)
spectator.set_transform(start_point)
leader.set_autopilot(True)

