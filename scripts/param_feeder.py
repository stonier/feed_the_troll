#!/usr/bin/env python
#
# License: MIT
#   https://raw.github.com/stonier/feed_the_troll/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Feed the troll a rosparam namespace!
"""

##############################################################################
# Imports
##############################################################################

import feed_the_troll
import rospy

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':

    rospy.init_node("feed_parameters")
    server_namespace = rospy.get_param("~server_namespace", "")
    configuration_namespace = rospy.get_param("~configuration_namespace", "~parameters")
    feeder = feed_the_troll.feeders.ROSParameters(server_namespace=server_namespace)
    rospy.spin()
