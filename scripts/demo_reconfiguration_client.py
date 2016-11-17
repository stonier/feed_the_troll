#!/usr/bin/env python
#
# License: MIT
#   https://raw.githubusercontent.com/stonier/feed_the_troll/devel/LICENSE
#
##############################################################################
# Imports
##############################################################################

from dynamic_reconfigure.client import Client
import functools
import rospy
import termcolor


def config_callback(config, namespace):
    print("")
    termcolor.cprint("Reconfiguration Client Callback", 'yellow', attrs=['bold'])
    print("")
    termcolor.cprint("  Reconfigure Client", "green")
    print("    " + termcolor.colored("{0: <23}".format("Name"), 'cyan') + ": " + termcolor.colored("{0}".format("dudette"), 'yellow'))
    print("    " + termcolor.colored("{0: <23}".format("Namespace"), 'cyan') + ": " + termcolor.colored("{0}".format(namespace), 'yellow'))
    termcolor.cprint("    Parameters", "cyan")
    for k, v in config.iteritems():
        if k != "groups":
            print("      " + termcolor.colored("{0: <21}".format(k), 'cyan') + ": " + termcolor.colored("{0}".format(v), 'yellow'))
    print("")

if __name__ == '__main__':
    rospy.init_node("reconfiguration_client")
    namespace = rospy.get_param("~name", "dudette")
    client = Client(name=namespace, config_callback=functools.partial(config_callback, namespace=namespace))
    rospy.spin()
