#
# License: MIT
#
##############################################################################
# Description
##############################################################################

"""
.. module:: servers
   :platform: Unix
   :synopsis: Ready to rumble servers

Various servers tailor made in the feeder-troll style to suit various purposes.
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
    Ordinarily you run a dynamic reconfigure server in the
    node of your choice, however this can be awkward if you need to share
    dynamic reconfigure parameter(s) amongst several nodes. Who should take
    responsibility for serving it? Or do you have duplicates?

    This standalone node volunteers for that and also allows you to
    reconfigure it at runtime, so as parts of your system go up and down,
    the collection of served dynamic reconfigure parameters will go up
    and down accordingly. Reconfigure your dynamic reconfiguration!

    To use, just launch this node:

    .. code-block:: xml

    .. literalinclude:: ../launch/demo_reconfiguration_server.launch
       :language: xml

    feed it using this package's parameter feeder:

    .. literalinclude:: ../launch/demo_reconfiguration_feeder.launch
       :language: xml

    with some parameterisation to fire up the requisite reconfigure servers (note this can be
    launched and torn down anytime and the server will construct/destruct the reconfigure servers
    in sync):

    .. literalinclude:: ../parameters/demo_reconfiguration.yaml

    The yaml parameters used above should be fairly self-explanatory.

    :ivar debug: print verbose debugging information on loading/unloading/updating
    """
    def __init__(self):
        """
        """
        self.troll = feed_the_troll.trolls.ROSParameters(
            loading_handler=self._load,
            unloading_handler=self._unload
        )
        self.reconfigure_servers = {}
        self.debug = rospy.get_param("~debug", False)

    def _load(self, unique_identifier, namespace):
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

    def _unload(self, unique_identifier, namespace):
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
