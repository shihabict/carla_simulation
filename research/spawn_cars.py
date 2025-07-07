import carla
import random
import cv2
import numpy as np


# connect to the simulator
client = carla.Client('localhost', 2000)
# load predefine town from carla
client.load_world('Town05')
