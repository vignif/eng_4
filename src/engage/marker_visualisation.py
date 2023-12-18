import rospy
from visualization_msgs.msg import Marker 
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

class MarkerMaker:
    skeleton_pairs = [
            ("r_ear","r_eye"),
            ("r_eye","nose"),
            ("l_ear","l_eye"),
            ("l_eye","nose"),
            ("nose","neck"),
            ("neck","r_sho"),
            ("neck","l_sho"),
            ("r_sho","r_elb"),
            ("r_elb","r_wri"),
            ("l_sho","l_elb"),
            ("l_elb","l_wri"),
            ("r_sho","r_hip"),
            ("l_sho","l_hip"),
            ("r_hip","l_hip"),
            ("r_hip","r_knee"),
            ("r_knee","r_ank"),
            ("l_hip","l_knee"),
            ("l_knee","l_ank"),
        ]
    
    skeleton_pairs_indices = [
            (16,14),
            (14,0),
            (17,15),
            (15,0),
            (0,1),
            (1,2),
            (1,5),
            (2,3),
            (3,4),
            (5,6),
            (6,7),
            (2,8),
            (5,11),
            (8,11),
            (8,9),
            (9,10),
            (11,12),
            (12,13)
        ]

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
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            return marker
        
    def make_line_marker(p1,p2,body_id,marker_id,colour=[1,0,0],frame_id="world"):
        if p1 is not None and p2 is not None:
            line_color = ColorRGBA() 
            line_color.r = colour[0]
            line_color.g = colour[1]
            line_color.b = colour[2]
            line_color.a = 1.0
            start_point = Point()
            start_point.x = p1[0]
            start_point.y = p1[1]
            start_point.z = p1[2]
            end_point = Point()
            end_point.x = p2[0]
            end_point.y = p2[1]
            end_point.z = p2[2]

            scale = 0.01

            mk = Marker()
            mk.id = marker_id
            mk.type = Marker.LINE_STRIP
            mk.header.frame_id = frame_id
            mk.ns = body_id
            mk.lifetime = rospy.Duration(0.1)
            mk.action = 0
            mk.scale.x = scale
            mk.points.append(start_point)
            mk.points.append(end_point)
            mk.colors.append(line_color)
            mk.colors.append(line_color)

            mk.pose.orientation.x = 0
            mk.pose.orientation.y = 0
            mk.pose.orientation.z = 0
            mk.pose.orientation.w = 1
            mk.pose.position.x = 0
            mk.pose.position.y = 0
            mk.pose.position.z = 0

            return mk