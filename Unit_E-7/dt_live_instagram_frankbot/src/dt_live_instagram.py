#!/usr/bin/env python
import rospy
from dt_live_instagram_frankbot.instagram import Instagram
from sensor_msgs.msg import CompressedImage

class LiveInstagram(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Read parameters
        filters = self.setupParameter("~filter", "")
        self.instagram = Instagram(filters)
        # Set the topics
        filters = filters.replace("-", "_")
        filters = filters.replace(":", "__")
        topic = "/frankbot/camera_node/image"
        compressed = "/compressed"
        topic_in = topic + compressed
        topic_out = topic + "/" + filters + compressed
        rospy.loginfo("[%s] Subscribing to topic %s." %(self.node_name, topic_in))
        rospy.loginfo("[%s] Publishing to topic %s." %(self.node_name, topic_out))

        # Setup publisher
        self.pub_filteredImg = rospy.Publisher(topic_out, CompressedImage, queue_size = 1)
        # Setup subscriber
        self.sub_img = rospy.Subscriber(topic_in, CompressedImage, self.cbTopic)

        rospy.loginfo("[%s] Initialized." %(self.node_name))


    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name, param_name, value))
        return value


    def cbTopic(self, msg_in):
        # Filter and publish the image
        msg_out = self.instagram.processMsg(msg_in)
        self.pub_filteredImg.publish(msg_out)


    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('dt_live_instagram_frankbot_node', anonymous=False)

    # Create the NodeName object
    node = LiveInstagram()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()