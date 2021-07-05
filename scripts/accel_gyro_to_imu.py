import numpy as np
import rospy
import threading

from std_msgs.msg import String
from sensor_msgs.msg import Imu

imu = Imu()
lock = threading.Lock()

a_alpha = 0.1
a_filtered = np.array([0.0, 0.0, 0.0])

W_alpha = 0.1
W_filtered = np.array([0.0, 0.0, 0.0])


def talker():
    pub = rospy.Publisher('/camera/imu/data_raw', Imu, queue_size=10)
    rate = rospy.Rate(63)
    
    while not rospy.is_shutdown():
        pub.publish(imu)
        rate.sleep()


def callback_accel(data):

    global a_filtered

    a = data.linear_acceleration
    a_raw = np.array([a.x, a.y, a.z])
    a_filtered = (1.0 - a_alpha) * a_filtered + a_alpha * a_raw

    with lock:
        imu.header.frame_id = data.header.frame_id
        imu.header.stamp = data.header.stamp
        imu.linear_acceleration.x = a_filtered[0]
        imu.linear_acceleration.y = a_filtered[1]
        imu.linear_acceleration.z = a_filtered[2]


def callback_gyro(data):

    global W_filtered

    W = data.angular_velocity
    W_raw = np.array([W.x, W.y, W.z])
    W_filtered = (1.0 - W_alpha) * W_filtered + W_alpha * W_raw

    with lock:
        imu.header.frame_id = data.header.frame_id
        imu.header.stamp = data.header.stamp
        imu.angular_velocity.x = W_filtered[0]
        imu.angular_velocity.y = W_filtered[1]
        imu.angular_velocity.z = W_filtered[2]


def listener():
    rospy.Subscriber('/device_0/sensor_2/Accel_0/imu/data',
                     Imu, callback_accel)
    rospy.Subscriber('/device_0/sensor_2/Gyro_0/imu/data',
                     Imu, callback_gyro)


if __name__ == '__main__':
    rospy.init_node('imu_intermediate_node', anonymous=True)

    try:
        listener()
        talker()
    except rospy.ROSInterruptException:
        pass
