#!/usr/bin/env python
#
# License: MIT
#   https://raw.github.com/stonier/feed_the_troll/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Test harness for reconfiguration testing.
"""

##############################################################################
# Imports
##############################################################################

import rospkg
import rospy
import unittest
import rostest
#from . import test_classes
from test_classes import TestLoading

##############################################################################
# Suite
##############################################################################

class TestReconfigurationSuite(unittest.TestSuite):

    def __init__(self):
        super(TestReconfigurationSuite, self).__init__()
        for method in dir(TestLoading):
            if method.startswith("test"):
                self.addTest(TestLoading(method))

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':

    rospy.init_node("test_reconfiguration")
    # args are
    #  - <package name> : to store test results in
    #  - <test_name> : filename for the results (e.g. <package_name>/TEST-<test_name>.xml
    #  - <class name> : actual test class to execute
    rostest.rosrun('feed_the_troll', 'test_reconfiguration', 'test_suite.TestReconfigurationSuite')
