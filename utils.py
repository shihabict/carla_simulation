import carla
import numpy as np
import cv2

def load_xodr(map_path):
    with open(map_path, "r") as f:
        xodr_content = f.read()
    return xodr_content

def create_world_from_custom_map(client,xodr_content):
    world = client.generate_opendrive_world(xodr_content)
    return world

def set_camera_view(camera_pos_z,camera_poz_x,world,attach_to,window_width=360,window_height=360):
    # camera mount offset on the car - you can tweak these to each car to avoid any parts of the car being in the view
    camera_pos_z = 1.6  # this means 1.6m up from the ground
    camera_poz_x = 0.9  # this is 0.9m forward

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
