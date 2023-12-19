import rospy
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import String
import time

test_pub = rospy.Publisher("/test_pub",String,queue_size=1)

def print_time(name,msg):
    print("{}:{}".format(name,msg.header.stamp))

def rgb_callback(msg):
    print_time("RGB IMG",msg)
    str_msg = String()
    str_msg.data = "Hello at {}".format(msg.header.stamp)
    test_pub.publish(str_msg)
    print(str_msg)

def dep_callback(msg):
    print_time("DEP IMG",msg)

def rgb_inf_callback(msg):
    print_time("RGB DAT",msg)

def dep_inf_callback(msg):
    print_time("DEP DAT",msg)

def synch_callback(rgb_img,dep_img,rgb_inf,dep_inf):
    print_time("RGB IMG",rgb_img)
    print_time("DEP IMG",dep_img)
    #print_time("RGB DAT",rgb_inf)
    #print_time("DEP DAT",dep_inf)
    print("\n")

rospy.init_node("TestNode", anonymous=True)

rgb_img_sub = rospy.Subscriber("/head_front_camera/color/image_raw",Image,rgb_callback,queue_size=1)
dep_img_sub = rospy.Subscriber("/head_front_camera/depth/image_rect_raw",Image,dep_callback,queue_size=1)
rgb_inf_sub = rospy.Subscriber("/head_front_camera/color/camera_info",CameraInfo,rgb_inf_callback,queue_size=1)
dep_inf_sub = rospy.Subscriber("/head_front_camera/depth/camera_info",CameraInfo,dep_inf_callback,queue_size=1)

'''
rgb_img_sub = Subscriber("/head_front_camera/color/image_raw",Image,queue_size=1)
dep_img_sub = Subscriber("/head_front_camera/depth/image_rect_raw",Image,queue_size=1)
rgb_inf_sub = Subscriber("/head_front_camera/color/camera_info",CameraInfo,queue_size=1)
dep_inf_sub = Subscriber("/head_front_camera/depth/camera_info",CameraInfo,queue_size=1)

synch_sub = ApproximateTimeSynchronizer(
            [
                rgb_img_sub,
                dep_img_sub,
                rgb_inf_sub,
                dep_inf_sub
            ],
            10,
            15,
        )
synch_sub.registerCallback(synch_callback)
'''

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    rate.sleep()

