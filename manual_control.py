import carla
import random
import pygame

# Connect to CARLA
client = carla.Client('localhost', 2000)
world = client.load_world('Town01')

# Set weather
weather = carla.WeatherParameters(
    cloudiness=0.0,
    precipitation=0.0,
    sun_altitude_angle=10.0,
    sun_azimuth_angle=70.0,
    precipitation_deposits=0.0,
    wind_intensity=0.0,
    fog_density=0.0,
    wetness=0.0, 
)
world.set_weather(weather)

# Spawn ego vehicle
bp_lib = world.get_blueprint_library() 
spawn_points = world.get_map().get_spawn_points()
vehicle_bp = bp_lib.find('vehicle.audi.etron')
ego_vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[79])

# Setup spectator view (initial only)
spectator = world.get_spectator()
initial_transform = carla.Transform(
    ego_vehicle.get_transform().transform(carla.Location(x=-4, z=2.5)),
    ego_vehicle.get_transform().rotation
)
spectator.set_transform(initial_transform)

# Spawn NPC vehicles
for i in range(200):  
    vehicle_bp = random.choice(bp_lib.filter('vehicle')) 
    npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

# Enable autopilot for NPCs
for v in world.get_actors().filter('*vehicle*'): 
    v.set_autopilot(True) 
ego_vehicle.set_autopilot(True)

# Initialize pygame
pygame.init() 
size = (640, 480)
pygame.display.set_caption("CARLA Manual Control")
screen = pygame.display.set_mode(size)

# Control loop
control = carla.VehicleControl()
clock = pygame.time.Clock()
done = False

while not done:
    keys = pygame.key.get_pressed()

    if keys[pygame.K_UP] or keys[pygame.K_w]:
        control.throttle = min(control.throttle + 0.05, 1.0)
    else:
        control.throttle = 0.0

    if keys[pygame.K_DOWN] or keys[pygame.K_s]:
        control.brake = min(control.brake + 0.2, 1.0)
    else:
        control.brake = 0.0

    if keys[pygame.K_LEFT] or keys[pygame.K_a]:
        control.steer = max(control.steer - 0.05, -1.0)
    elif keys[pygame.K_RIGHT] or keys[pygame.K_d]:
        control.steer = min(control.steer + 0.05, 1.0)
    else:
        control.steer = 0.0

    control.hand_brake = keys[pygame.K_SPACE]

    # Apply control and advance simulation
    ego_vehicle.apply_control(control)
    world.tick()

    # Update camera position to follow vehicle
    ego_transform = ego_vehicle.get_transform()
    camera_offset = carla.Location(x=-4.5, z=2.5)
    spectator_transform = carla.Transform(
        ego_transform.transform(camera_offset),
        ego_transform.rotation
    )
    spectator.set_transform(spectator_transform)

    # Update display and check for quit
    pygame.display.flip()
    pygame.display.update()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True

    clock.tick(60)  # 60 FPS
