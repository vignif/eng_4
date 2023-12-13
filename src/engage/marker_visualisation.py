import rospy
from visualization_msgs.msg import Marker 

class MarkerMaker:
    def make_marker_sphere(position,body_id,marker_id,colour=[0,0,0],frame_id="world"):
        if position is not None:
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.id = marker_id
            marker.ns = body_id
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.lifetime = rospy.Duration(0.1)
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = colour[0]
            marker.color.g = colour[1]
            marker.color.b = colour[2]
            marker.color.a = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            return marker