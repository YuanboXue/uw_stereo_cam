import numpy as np
import cv2
import os
import rospy
from cv_bridge import CvBridge
import ros_numpy

from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from rospy.numpy_msg import numpy_msg

pub_left = rospy.Publisher('host_data_left',Image,queue_size=1)
pub_right = rospy.Publisher('host_data_right',Image,queue_size=1)

def callback(data):
    np_arr = np.fromstring(data.data, np.uint8)
    img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    h,w,c = img_np.shape
    img_np_left = img_np[:,:w//2, :]
    img_np_right = img_np[:,w//2:,:]
    
    #msg_left = Image()
    #msg_right = Image()
    #msg_left.header.stamp = data.header.stamp
    #msg_left.format = "jpeg"
    #msg_left.data = np.array(cv2.imencode('.jpg', img_np_left)[1]).tostring()
    #msg_right.header.stamp = data.header.stamp
    #msg_right.format = "jpeg"
    #msg_right.data = np.array(cv2.imencode('.jpg', img_np_right)[1]).tostring()
    msg_left = ros_numpy.msgify(Image,img_np_left,encoding='rgb8')
    msg_right = ros_numpy.msgify(Image,img_np_right,encoding='rgb8')
    pub_left.publish(msg_left)
    pub_right.publish(msg_right)
    #rospy.loginfo(rospy.get_caller_id() + "I heard data:{}".format(img_np.shape))
    

def listener():
    rospy.init_node('listener', anonymous=True)
    #rospy.Subscriber("/stereo_camera", numpy_msg(Image), callback)
    rospy.Subscriber("/stereo_camera/compressed", CompressedImage, callback, queue_size = 1)
    rospy.spin()

    
    
if __name__ == '__main__':
    listener()