#!/usr/bin/env python
#
# License: MIT
#   https://raw.githubusercontent.com/stonier/feed_the_troll/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
A troll lurking, waiting for parameter feeding.
"""

##############################################################################
# Imports
##############################################################################

import feed_the_troll
import rocon_console.console as console
import rospy

##############################################################################
# Classes
##############################################################################


class Server(object):
    def __init__(self):
        self.troll = feed_the_troll.trolls.ROSParameters(
            loading_handler=self.load,
            unloading_handler=self.unload
        )

    def load(self, unique_identifier, namespace):
        """
        :param uuid.UUID unique_identifier:
        :param str namespace: root namespace for configuration on the parameter server
        """
        print("Troll: loading [{0}][{1}]".format(unique_identifier, namespace))
        print("\n{0}".format(self.troll))
        parameters = rospy.get_param(namespace)
        print(console.green + "Loaded Parameters" + console.reset)
        for k, v in parameters.iteritems():
            print("  " + console.cyan + "{0}".format(k) + console.reset + ": " + console.yellow + "{0}".format(v) + console.reset)
        return (True, "Success")

    def unload(self, unique_identifier, namespace):
        """
        :param uuid.UUID unique_identifier:
        """
        print("Troll: unloading [{0}][{1}]".format(unique_identifier, namespace))
        parameters = rospy.get_param(namespace)
        print(console.green + "Unloaded Parameters" + console.reset)
        for k, v in parameters.iteritems():
            print("  " + console.cyan + "{0}".format(k) + console.reset + ": " + console.yellow + "{0}".format(v) + console.reset)
        return (True, "Success")

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':

    rospy.init_node("troll")
    server = Server()
    rospy.spin()
