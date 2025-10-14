# In your CARLA Python script:
# from carla import OpenDriveParser
import carla

from research.utils import draw_route_plan

xodr_file = "../routes/clean_road.xodr"
with open(xodr_file, 'r') as f:
    opendrive_data = f.read()



client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
vertex_distance = 2.0  # in meters
max_road_length = 50.0 # in meters
wall_height = 1.0      # in meters
extra_width = 0.6      # in meters
world = client.generate_opendrive_world(
    opendrive_data,
    carla.OpendriveGenerationParameters(
        vertex_distance=vertex_distance,
        max_road_length=max_road_length,
        wall_height=wall_height,
        additional_width=extra_width,
        smooth_junctions=True,
        enable_mesh_visibility=True
    )
)
spawn_points = world.get_map().get_spawn_points()
#get the map
map = world.get_map()
roads = map.get_topology()
print(len(roads))
# get waypoints
map.get_waypoints()

draw_route_plan(map, world, spawn_points)