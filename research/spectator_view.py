#all imports
import carla #the sim library itself
import random #to pick random spawn point
import cv2
import numpy as np #in this example to change image representation - re-shaping

# connect to the sim
client = carla.Client('localhost', 2000)

# optional to load different towns
#client.set_timeout(15)
client.load_world('Town04')

#define environment/world and get possible places to spawn a car
world = client.get_world()
spawn_points = world.get_map().get_spawn_points()

#look for a blueprint of Mini car
vehicle_bp = world.get_blueprint_library().find('vehicle.tesla.model3')

#spawn a car in a random location

start_point = spawn_points[0]
vehicle = world.try_spawn_actor(vehicle_bp, start_point)

# move simulator view to the car
spectator = world.get_spectator()
start_point.location.z = start_point.location.z+1 #start_point was used to spawn the car but we move 1m up to avoid being on the floor
spectator.set_transform(start_point)

#send the car off on autopilot - this will leave the spectator
vehicle.set_autopilot(True)
#setting RGB Camera - this follow the approach explained in a Carla video
# link: https://www.youtube.com/watch?v=om8klsBj4rc&t=1184s

#camera mount offset on the car - you can tweak these to each car to avoid any parts of the car being in the view
CAMERA_POS_Z = 1.6 #this means 1.6m up from the ground
CAMERA_POS_X = 0.9 #this is 0.9m forward

camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '640') # this ratio works in CARLA 9.14 on Windows
camera_bp.set_attribute('image_size_y', '640')

camera_init_trans = carla.Transform(carla.Location(z=CAMERA_POS_Z,x=CAMERA_POS_X))
#this creates the camera in the sim
camera = world.spawn_actor(camera_bp,camera_init_trans,attach_to=vehicle)

def camera_callback(image,data_dict):
    data_dict['image'] = np.reshape(np.copy(image.raw_data),(image.height,image.width,4))

image_w = camera_bp.get_attribute('image_size_x').as_int()
image_h = camera_bp.get_attribute('image_size_y').as_int()

camera_data = {'image': np.zeros((image_h,image_w,4))}
# this actually opens a live stream from the camera
camera.listen(lambda image: camera_callback(image,camera_data))

# while True:
#
#     # Dispaly with imshow
#     cv2.imshow('All cameras', camera_data['image'])
#
#     # Break loop if user presses q
#     if cv2.waitKey(1) == ord('q'):
#         break
# cv2.destroyAllWindows()

#grab a snapshot from the camera an show in a pop-up window
# img = camera_data['image']
# cv2.imshow('RGB Camera',img)
# cv2.waitKey(0)

# clean up after yourself

# camera.stop() # this is the opposite of camera.listen
# for actor in world.get_actors().filter('*vehicle*'):
#     actor.destroy()
# for sensor in world.get_actors().filter('*sensor*'):
#     sensor.destroy()
#
# start_point = spawn_points[0]
# spectator = world.get_spectator()
# spectator.set_transform(start_point)
#
# spectator.set_transform(carla.Transform(carla.Location(x=-1085.286377, y=3112.225830, z=356.060608), carla.Rotation(pitch=1.648070, yaw=20.234367, roll=0.000000)))
#
#
# print(start_point)