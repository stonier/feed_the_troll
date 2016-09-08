#
# License: MIT
#
##############################################################################
# Description
##############################################################################

"""
.. module:: reconfiguration
   :platform: Unix
   :synopsis: Reconfigurable reconfiguration.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

"""

##############################################################################
# Imports
##############################################################################

import dynamic_reconfigure.server
import feed_the_troll
import importlib
import rospy
import termcolor

##############################################################################
# ReConfiguration
##############################################################################


class ReConfiguration(object):
    """
    Utilises the parameter troll to dynamically configure a collection of
    dynamic reconfigure servers.

    :ivar troll: the underlying parameter troll that handles the dynamic configuration of the server
    :ivar debug: print verbose debugging information on loading/unloading/updating
    """
    def __init__(self):
        """
        """
        self.troll = feed_the_troll.trolls.ROSParameters(
            loading_handler=self.load,
            unloading_handler=self.unload
        )
        self.reconfigure_servers = {}
        self.debug = rospy.get_param("~debug", False)

    def load(self, unique_identifier, namespace):
        """
        :param uuid.UUID unique_identifier:
        :param str namespace: root namespace for configuration on the parameter server
        """
        parameters = rospy.get_param(namespace)
        self._pretty_print_incoming("Reconfigure Loading", unique_identifier, namespace, parameters)
        for k, v in parameters.iteritems():
            reconfigure_module = importlib.import_module(v['module'])
            self.reconfigure_servers[k] = dynamic_reconfigure.server.Server(reconfigure_module,
                                                                            self.callback,
                                                                            namespace=v['namespace'])
        return (True, "Success")

    def unload(self, unique_identifier, namespace):
        """
        :param uuid.UUID unique_identifier:
        """
        parameters = rospy.get_param(namespace)
        self._pretty_print_incoming("Reconfigure Unloading", unique_identifier, namespace, parameters)
        for k, unused_v in parameters.iteritems():
            del self.reconfigure_servers[k]
        return (True, "Success")

    def callback(self, config, level):
        if self.debug:
            print("")
            termcolor.cprint("Reconfiguration Updating", 'white', attrs=['bold'])
            print("")
            for k, v in config.iteritems():
                print("  " + termcolor.colored("{0: <25}".format(k), 'cyan') + ": " + termcolor.colored("{0}".format(v), 'yellow'))
        return config

    def _pretty_print_incoming(self, title, unique_identifier, namespace, parameters):
        if self.debug:
            print("")
            termcolor.cprint(title, 'white', attrs=['bold'])
            print("")
            print("  " + termcolor.colored("{0: <15}".format('Feeder'), 'cyan') + ": " + termcolor.colored("{0}".format(unique_identifier), 'yellow') +
                  "-" + termcolor.colored("{0}".format(namespace), 'green'))
            for k, v in parameters.iteritems():
                print("  " + termcolor.colored("{0: <15}".format("Server"), 'cyan') + ": " + termcolor.colored("{0}".format(k), 'magenta') +
                      "-" + termcolor.colored("{0}".format(v['namespace']), 'yellow') + "-" + termcolor.colored("{0}".format(v['module']), 'green'))

    def spin(self):
        rospy.spin()
