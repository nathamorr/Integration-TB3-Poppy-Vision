#!/usr/bin/env python3

#
# Import packages
#
import cv2, os, rospy, sys, argparse, pathlib
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image as pil_image

from poppy_controllers.srv import GetImage
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

import tensorflow as tf
from object_detection.utils import label_map_util

#
# do some tensorflow low level stuff...
#
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'    # Suppress TensorFlow logging (1)
tf.get_logger().setLevel('ERROR')           # Suppress TensorFlow logging (2)

#
# parse command line arguments
#
parser = argparse.ArgumentParser(description="Uses a trained network to detect object in images")
parser.add_argument('-p', '--project', type=str, required=True, default = "",
                    help='project name.')
parser.add_argument('-s', '--path_to_saved_model', type=str, required=True, default="~/Documents/ROS4PRO/catkin_ws/src/tod_tf2/training/faces_cubes/faster_rcnn_resnet50_v1_640x640_coco17_tpu-8",
                    help='path to the "saved_model" directory.')
parser.add_argument('-n', '--nb_max_object', type=int, required=True, default=4,
                    help='number max of object to detect.')
parser.add_argument('-t', '--threshold', type=int, required=False, default=80,
                    help='Detection theshold (percent) to display bounding boxe.')
parser.add_argument('-v', '--verbose', action="count", default=True, help='wether to run in verbose mode or not')
args = parser.parse_args()

verbose = True if args.verbose else False

#
# Set useful names
#
#"~/Documents/ROS4PRO/catkin_ws/src/tod_tf2/training/faces_cubes/faster_rcnn_resnet50_v1_640x640_coco17_tpu-8"
PATH_TO_SAVED_MODEL = args.path_to_saved_model
PROJECT   = args.project
THRESHOLD  = args.threshold/100
NB_MAX_OBJ = args.nb_max_object

# Load saved model and build the detection function
print('Loading model...', end='')
detect_fn = tf.saved_model.load(PATH_TO_SAVED_MODEL)
print('Done!')

# Load label map data: 
PATH_TO_LABELS = os.path.join('./training', PROJECT, 'label_map.pbtxt')
category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)


##
## ROS stuff to complete...
##

## 1/ Wait for the ROS parameter /takeImage to become True.
## ...put your code hereafter:
## ...

takeImage = rospy.get_param("/takeImage")
while not takeImage:
    rospy.sleep(1)
    takeImage = rospy.get_param("/takeImage")
    print("nn | Attente de /takeImage == True\n")

## ...
## ...end of your code.


## 2/ Set the ROS parameter /takeImage to False.
## ...put your code hereafter:
## ...
rospy.set_param("/takeImage", False)
print("nn | /takeImage set to False\n")
## ...
## ...end of your code.


## 3/ Get the image from the robot camera using the /get_image ROS service
## and write it (cv2.write(...) ) as "image.png".
## HELP : see lines 7 to 11 in file tod_tf2/get_image_from_robot.py
## ...put your code hereafter:
## ...

#get_image = rospy.ServiceProxy("get_image", GetImage)
#response  = get_image()
#bridge    = CvBridge()
#image     = bridge.imgmsg_to_cv2(response.image)
#cv2.imwrite(f"image.png", image)


rospy.init_node("usb_cam_subscriber")

bridge    = CvBridge()
image = rospy.wait_for_message("/usb_cam/image_raw", Image)
print(image)
img = bridge.imgmsg_to_cv2(image)

cv2.imwrite(f"image.png", img)
print("nn | getImage fait\n")

## ...
## ...end of your code.


## 4/ Run the network inference to detect cube faces.
## HELP : see lines 120-150 in file tod_tf2/plot_object_detection.py
## ...put your code hereafter:
## ...

print('Running inference for image.png... ')
image_np     = np.array(pil_image.open("image.png"))
input_tensor = tf.convert_to_tensor(image_np)  # convert input needs to be a tensor
input_tensor = input_tensor[tf.newaxis, ...]   # the model expects an array of images: add an axis
detections   = detect_fn(input_tensor)         # make the detections of the objects

num_detections = int(detections.pop('num_detections'))
if num_detections > NB_MAX_OBJ: num_detections = NB_MAX_OBJ
detections = {key: value[0, :num_detections].numpy() for key, value in detections.items()}
detections['num_detections'] = num_detections
# detection_classes should be ints:
detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

print(detections['detection_classes'])
print(detections['detection_scores'])
print(detections['detection_boxes'])

#image_np_with_detections = image_np.copy()

#vis_utils.visualize_boxes_and_labels_on_image_array(
#    image_np_with_detections,
#    detections['detection_boxes'],
#    detections['detection_classes'],
#    detections['detection_scores'],
#    category_index,
#    line_thickness=3,
#    use_normalized_coordinates=True,
#    max_boxes_to_draw=4,
#    min_score_thresh=THRESHOLD,
#    agnostic_mode=False)
    
#plt.figure()
#plt.imshow(image_np_with_detections)
#plt.show()
## ...
## ...end of your code.


## 5/ Sort the lists returned by the network to arrange data  
## from the leftmost cube to the rightmost cube, based on the abcissa of 
## the top-left corner of the bounding boxes of the cubes
## HELP : see np.argsort(...) here https://numpy.org/doc/stable/reference/generated/numpy.argsort.html...
## ...put your code hereafter:
## ...

x_list = detections['detection_boxes'][:,1]
#Liste des indices
indexes = np.argsort(x_list)
list_label_sorted = detections['detection_classes'][indexes]
list_boxe_sorted  = detections['detection_boxes'][indexes]
list_score_sorted = detections['detection_scores'][indexes]

if verbose:
    print(list_label_sorted)
    print(list_score_sorted)
    print(list_boxe_sorted)

## ...
## ...end of your code.


## 6/ Loop in the list of labels to set the parameter /label;
## set the ROS parameter /robotReady to False and wait for 
## /robotReady to be True.
## ...put your code hereafter:
## ...

for label in list_label_sorted:
    
    print(f"set ROS param /label to {label}") 
    rospy.set_param("/label", int(label))
    
    print("set ROS param /robotReady to False") 
    rospy.set_param("/robotReady", False)

    robotReady = rospy.get_param("/robotReady")
    while not robotReady:
        rospy.sleep(1)
        robotReady = rospy.get_param("/robotReady")

## ...
## ...end of your code.

