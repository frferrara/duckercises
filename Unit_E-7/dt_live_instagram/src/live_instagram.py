#!/usr/bin/env python
import rospy
from dt_live_instagram.instagram import Instagram
from std_msgs.msg import String #Imports msg

class LiveInstagram(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Read parameters
        filters = self.setupParameter("~filters", "")
        self.instagram = Instagram(filters)
        # Setup publishers
        self.pub_filteredImg = rospy.Publisher("write topic here/filtered/compressed", String, queue_size=1)
        # Setup subscriber
        self.sub_img = rospy.Subscriber("write topic here/compressed", String, self.cbTopic)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name, param_name, value))
        return value

    def cbTopic(self, msg):
        # Filter and publish the image
        msg_out = self.instagram.processMsg(msg)
        self.pub_filteredImg.publish(msg_out)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('talker', anonymous=False)

    # Create the NodeName object
    node = Talker()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()