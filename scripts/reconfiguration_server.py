#!/usr/bin/env python
#
# License: MIT
#   https://raw.githubusercontent.com/stonier/feed_the_troll/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################
"""
Reconfigurable reconfigure server.
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
    rospy.init_node("reconfiguration")
    troll = feed_the_troll.servers.ReConfiguration()
    troll.spin()
