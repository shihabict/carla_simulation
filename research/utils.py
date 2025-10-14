import os.path

import carla
import numpy as np
import cv2
import math
import time
import pandas as pd


def load_xodr(map_path):
    with open(map_path, "r") as f:
        xodr_content = f.read()
    return xodr_content

def create_world_from_custom_map(client,xodr_content):
    world = client.generate_opendrive_world(xodr_content)
    return world

def set_camera_view(camera_pos_z,camera_poz_x,world,attach_to,window_width=360,window_height=360):
    # camera mount offset on the car - you can tweak these to each car to avoid any parts of the car being in the view
    camera_pos_z = camera_pos_z  # this means 1.6m up from the ground
    camera_poz_x = camera_poz_x  # this is 0.9m forward

    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x',
                            '360')  # 16:9 type ratios work in CARLA 9.14 on Windows so we change it to something different here
    camera_bp.set_attribute('image_size_y', '360')

    camera_init_trans = carla.Transform(carla.Location(z=camera_pos_z, x=camera_poz_x))
    # this creates the camera in the sim
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=attach_to)

    def camera_callback(image, data_dict):
        data_dict['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

    image_w = camera_bp.get_attribute('image_size_x').as_int()
    image_h = camera_bp.get_attribute('image_size_y').as_int()

    camera_data = {'image': np.zeros((image_h, image_w, 4))}
    # this actually opens a live stream from the camera
    camera.listen(lambda image: camera_callback(image, camera_data))

    # this will open RGB camera window, select it from the task bar if it does not pop up
    # you will see the result - if camera works properly
    while True:

        # Dispaly with imshow
        cv2.imshow('All cameras', camera_data['image'])

        # Break loop if user presses q
        if cv2.waitKey(1) == ord('q'):
            break
    cv2.destroyAllWindows()

#function to claculate sides (x,y) of a right-angle triangle

def calculate_sides(hypotenuse, angle):

      """
      Calculates the two sides of a right triangle given the hypotenuse and an angle.

      Args:
        hypotenuse: The length of the hypotenuse of the triangle.
        represents the distance we need to be from the car
        angle: The angle of the triangle in degrees.
        represents the yaw angle of the car we need to be aligned with

      Returns:
        A tuple containing the lengths of the two sides of the triangle.
        which are delta x and y
      """

      # Convert the angle to radians
      angle_radians = math.radians(angle)

      # Calculate the opposite side using the sine function
      opposite_side = hypotenuse * math.sin(angle_radians)

      # Calculate the adjacent side using the cosine function
      adjacent_side = hypotenuse * math.cos(angle_radians)

      return opposite_side, adjacent_side

def get_spectator_pos(vehicle,spectator):
    # follow the car
    # here we subtract the delta x and y to be behind
    metres_distance = 5
    vehicle_transform = vehicle.get_transform()
    y, x = calculate_sides(metres_distance, vehicle_transform.rotation.yaw)

    spectator_pos = carla.Transform(vehicle_transform.location + carla.Location(x=-x, y=-y, z=5),
                                    carla.Rotation(yaw=vehicle_transform.rotation.yaw, pitch=-25))
    spectator.set_transform(spectator_pos)
    return spectator

def load_csv(csv_path):
    if os.path.exists(csv_path):
        csv_data = pd.read_csv(csv_path)
        return csv_data
    return None

def load_speed_profile(csv_path):
    """
    Loads speed profile from CSV, skips initial zeros, converts km/h to m/s.
    Returns: list of (relative_time, speed_mps)
    """
    df = pd.read_csv(csv_path)
    speeds = df['Message'].values
    times = df['Time'].values

    # Find the index where speed first becomes nonzero
    start_idx = next((i for i, s in enumerate(speeds) if s > 0), None)
    if start_idx is None:
        raise ValueError("No nonzero speed found in the CSV!")

    # Convert speed to m/s
    speeds_mps = speeds[start_idx:] / 3.6
    times = times[start_idx:]

    # Make all timestamps relative to the start
    rel_times = times - times[0]

    return list(zip(rel_times, speeds_mps))

def apply_speed_profile_realtime(vehicle, speed_profile):
    """
    Applies the speed profile to the vehicle using real CSV timing.
    """
    start_wall_time = time.time()
    for i, (rel_time, target_speed) in enumerate(speed_profile):
        # Wait until real time matches CSV time
        now = time.time()
        elapsed = now - start_wall_time
        wait_time = rel_time - elapsed
        if wait_time > 0:
            time.sleep(wait_time)

        # Control logic (simple P controller, can replace with PID)
        v = vehicle.get_velocity()
        current_speed = (v.x**2 + v.y**2 + v.z**2)**0.5
        error = target_speed - current_speed
        control = carla.VehicleControl()
        control.throttle = min(max(error * 0.5, 0.0), 1.0)
        control.brake = 0.0 if error >= 0 else min(abs(error) * 0.5, 1.0)
        vehicle.apply_control(control)

        # Optional: print current vs. target speed for debugging
        # print(f"t={rel_time:.2f}s, set {target_speed:.2f} m/s, current {current_speed:.2f} m/s")

class CustomVector3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def round_components(self, decimals=0):
        self.x = round(self.x, decimals)
        self.y = round(self.y, decimals)
        self.z = round(self.z, decimals)

    def __repr__(self):
        return f"Vector3D({self.x}, {self.y}, {self.z})"

def truncate_float(number, digits=3):
    factor = 10.0 ** digits
    return int(number * factor) / factor

# draw road waypoints
# import some code coming with the sim

def draw_route_plan(map, world, spawn_points):
    import sys
    sys.path.append('/home/paridhi/Downloads/CARLA_0.9.15/PythonAPI/carla')
    from agents.navigation.global_route_planner import GlobalRoutePlanner
    smapling_resolution = 2.0  # meters
    grp = GlobalRoutePlanner(map, smapling_resolution)

    str_point = carla.Location(x=100, y=0, z=0)
    end_point = carla.Location(x=str_point.x+500 , y=str_point.y  , z=str_point.z)

    route = grp.trace_route(str_point, end_point)

    for waypoint in route:
        world.debug.draw_string(waypoint[0].transform.location, '-', draw_shadow=False,
                                color=carla.Color(r=0, g=0, b=255), life_time=120.0,
                                persistent_lines=True)