import cv2
import numpy as np
import pdb
import rosbag
import rospy
import threading

from sensor_msgs.msg import Imu, Image

from datetime import date, datetime
from keras_segmentation.pretrained import pspnet_50_ADE_20K, pspnet_101_cityscapes, pspnet_101_voc12

model = pspnet_101_voc12()
lock = threading.Lock()

scale = 0.1
h = 480
w = 640

bag = rosbag.Bag('test.bag', 'w')
image_topic = '/device_0/sensor_1/Color_0/image/data'

pub = rospy.Publisher('/camera/image/data_cropped', Image, queue_size=10)


def callback_image(frame):
    h_scaled = int(scale * h)
    w_scaled = int(scale * w)
    frame_scaled = cv2.resize(frame, (w_scaled, h_scaled))
    out = model.predict_segmentation(inp=frame_scaled)
    mask = (out / float(np.max(out)) * 255.0).astype(np.uint8)
    # mask = cv2.bitwise_not(mask)

    h_mask, w_mask = mask.shape
    mask_resized = cv2.resize(mask, (w, h))
    cropped = cv2.bitwise_and(frame, frame, mask=mask_resized)

    pub.publish(cropped)



def listener():
    rospy.Subscriber(image_topic, Image, callback_image)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('section_bag_node', anonymous=True)

    listener()
