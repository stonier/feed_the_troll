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

import feed_the_troll
import os
import rosgraph
import rosparam
import rospkg
import rospy
import rostest
import termcolor
import unittest
import yaml

##############################################################################
# YAML Strings
##############################################################################

def get_valid_yaml():
    rospack = rospkg.RosPack()
    feed_the_troll_path = rospack.get_path('feed_the_troll')
    yaml_file_path = os.path.join(feed_the_troll_path, 'parameters', 'demo_reconfiguration.yaml')
    reconfiguration_yaml = yaml.load(open(yaml_file_path))
    return reconfiguration_yaml

def pretty_print_banner(title):
    print("")
    termcolor.cprint('*'*79, 'white', attrs=['bold'])
    termcolor.cprint("* {0}".format(title), 'white', attrs=['bold'])
    termcolor.cprint('*'*79, 'white', attrs=['bold'])
    print("")

##############################################################################
# Main
##############################################################################

class TestReConfiguration(unittest.TestCase):

    def setUp(self):
        self.yaml = get_valid_yaml()

    def test_valid_loading(self):
        pretty_print_banner("Loading - Valid")
        rosparam.upload_params(rospy.names.resolve_name("~").rstrip('/'), self.yaml)
        feeder = feed_the_troll.feeders.ROSParameters(server_namespace=self.yaml["server_namespace"],
                                                      add_shutdown_hook=False
                                                      )
        self.assertTrue(feeder, "Yaml Valid")
        feeder.shutdown()
        rospy.delete_param(rospy.names.resolve_name("~").rstrip('/'))

    def test_invalid_module_loading(self):
        pretty_print_banner("Loading - Invalid Module")
        broken_yaml = self.yaml.copy()
        broken_yaml['parameters']['dude']['module'] = 'feed_the_troll.cfg.Demo'
        rosparam.upload_params(rospy.names.resolve_name("~").rstrip('/'), broken_yaml)
        feeder = feed_the_troll.feeders.ROSParameters(server_namespace=self.yaml["server_namespace"])
        self.assertTrue(feeder, "Invalid Dynamic Reconfigure Type")
        rospy.delete_param(rospy.names.resolve_name("~").rstrip('/'))

    def test_no_module_parameter_loading(self):
        pretty_print_banner("Loading - No Module Parameter")
        broken_yaml = self.yaml.copy()
        del broken_yaml['parameters']['dude']['module']
        rosparam.upload_params(rospy.names.resolve_name("~").rstrip('/'), broken_yaml)
        feeder = feed_the_troll.feeders.ROSParameters(server_namespace=self.yaml["server_namespace"])
        self.assertTrue(feeder, "No Dynamic Reconfigure Parameter")
        rospy.delete_param(rospy.names.resolve_name("~").rstrip('/'))


##############################################################################
# Main
##############################################################################

if __name__ == '__main__':

    rospy.init_node("test_reconfiguration")
    # args are
    #  - <package name> : to store test results in
    #  - <test_name> : filename for the results (e.g. <package_name>/TEST-<test_name>.xml
    #  - <class name> : actual test class to execute
    rostest.rosrun('feed_the_troll', 'test_reconfiguration', TestReConfiguration)
