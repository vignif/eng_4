import rospy
from opendr.engine.data import Image
from opendr.perception.pose_estimation import draw
from opendr.perception.pose_estimation import LightweightOpenPoseLearner

rospy.init_node("Test", anonymous=True)
print("Hello")
