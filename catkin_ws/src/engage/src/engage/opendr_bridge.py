# Copyright 2020-2023 OpenDR European Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This file is copied and adapted from OpenDR's ROS 1 workspace

from opendr.engine.data import Image

import numpy as np


from rospy.rostime import Time
from cv_bridge import CvBridge

from std_msgs.msg import Header
from sensor_msgs.msg import Image as ImageMsg

class ROSBridge:
    """
    This class provides an interface to convert OpenDR data types and targets into ROS-compatible ones similar to CvBridge.
    For each data type X two methods are provided:
    from_ros_X: which converts the ROS equivalent of X into OpenDR data type
    to_ros_X: which converts the OpenDR data type into the ROS equivalent of X
    """

    def __init__(self):
        self._cv_bridge = CvBridge()

    def from_ros_image(self, message: ImageMsg, encoding: str='passthrough') -> Image:
        """
        Converts a ROS image message into an OpenDR image
        :param message: ROS image to be converted
        :type message: sensor_msgs.msg.Image
        :param encoding: encoding to be used for the conversion (inherited from CvBridge)
        :type encoding: str
        :return: OpenDR image (RGB)
        :rtype: engine.data.Image
        """
        cv_image = self._cv_bridge.imgmsg_to_cv2(message, desired_encoding=encoding)
        image = Image(np.asarray(cv_image, dtype=np.uint8))
        return image

    def to_ros_image(self,
                     image: Image,
                     encoding: str='passthrough',
                     frame_id: str = None,
                     time: Time = None) -> ImageMsg:
        """
        Converts an OpenDR image into a ROS image message
        :param image: OpenDR image to be converted
        :type image: engine.data.Image
        :param encoding: encoding to be used for the conversion (inherited from CvBridge)
        :type encoding: str
        :param frame_id: frame id of the image
        :type frame_id: str
        :param time: time of the image
        :type time: rospy.rostime.Time
        :return: ROS image
        :rtype: sensor_msgs.msg.Image
        """
        # Convert from the OpenDR standard (CHW/RGB) to OpenCV standard (HWC/BGR)
        header = Header()
        if frame_id is not None:
            header.frame_id = frame_id
        if time is not None:
            header.stamp = time
        message = self._cv_bridge.cv2_to_imgmsg(image.opencv(), encoding=encoding, header=header)
        return message