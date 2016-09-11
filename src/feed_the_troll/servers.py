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
import rosgraph
import rospy
import termcolor
import threading

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

    In short, **you simply need to create your own yaml and launchers** - no
    python coding necessary.

    :ivar debug: print verbose debugging information on loading/unloading/updating
    """
    def __init__(self):
        """
        """
        self.troll = feed_the_troll.trolls.ROSParameters(
            loading_handler=self._load,
            unloading_handler=self._unload
        )
        self.guard = threading.Lock()
        self.reconfigure_servers = {}
        self.debug = rospy.get_param("~debug", False)

    def _load(self, unique_identifier, namespace):
        """
        :param uuid.UUID unique_identifier:
        :param str namespace: root namespace for configuration on the parameter server
        """
        try:
            parameters = rospy.get_param(namespace)
        except KeyError:
            error_message = "could not retrieve parameters for configuration [{0}]".format(namespace)
            rospy.logerr("Reconfiguration: {0}".format(error_message))
            return (False, error_message)

        self._pretty_print_incoming("Reconfigure Loading", unique_identifier, namespace, parameters)
        error_messages = []
        # checks - parse through every item looking for flaws before doing anything
        with self.guard:
            for k, v in parameters.iteritems():
                if k in self.reconfigure_servers:
                    error_messages.append("this reconfigure server is already being served [{0}]".format(k))
                    continue
                if 'module' not in v:
                    error_messages.append("no dynamic reconfigure 'module' specified in the parameters feed to the server (e.g. 'feed_the_troll.cfg.DemoConfig']")
                    continue
                try:
                    reconfigure_module = importlib.import_module(v['module'])
                except ImportError:
                    error_messages.append("could not import dynamic reconfigure module [{0}]".format(v['module']))
                    continue
        if error_messages:
            rospy.logerr("Reconfiguration: errors loading the passed parameterisations")
            for message in error_messages:
                rospy.logerr("               : {0}".format(message))
            return (False, ', '.join(error_messages))
        # setup
        with self.guard:
            for k, v in parameters.iteritems():
                reconfigure_module = importlib.import_module(v['module'])
                if 'overrides' in v:
                    rospy.set_param(rosgraph.names.ns_join("~", k), v['overrides'])
                self.reconfigure_servers[k] = dynamic_reconfigure.server.Server(
                    reconfigure_module,
                    self.callback,
                    namespace=rosgraph.names.ns_join("~", k)
                )
        return (True, "Success")

    def _unload(self, unique_identifier, namespace):
        """
        :param uuid.UUID unique_identifier:
        """
        error_messages = []
        with self.guard:
            parameters = rospy.get_param(namespace)
            self._pretty_print_incoming("Reconfigure Unloading", unique_identifier, namespace, parameters)
            for k, unused_v in parameters.iteritems():
                reflected_parameters = rosgraph.names.ns_join("~", k)
                if rospy.has_param(reflected_parameters):
                    rospy.delete_param(reflected_parameters)
                if k not in self.reconfigure_servers:
                    error_messages.append("could not find server to unload [{0}]".format(k))
                    continue
                server = self.reconfigure_servers.pop(k)
                server.set_service.shutdown()
                server.descr_topic.unregister()
                server.update_topic.unregister()
                del server.set_service
                del server
        if error_messages:
            rospy.logerr("Reconfiguration: errors while unloading")
            for message in error_messages:
                rospy.logerr("               : {0}".format(message))
            return (False, ', '.join(error_messages))
        else:
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
            print("  " + termcolor.colored("{0: <25}".format('Feeder'), 'cyan') + ": " + termcolor.colored("{0}".format(namespace), 'yellow') +
                  "-" + termcolor.colored("{0}".format(unique_identifier), 'yellow'))
            for k, v in parameters.iteritems():
                print("  " + termcolor.colored("{0}".format("Reconfigure Server"), 'green'))
                print("    " + termcolor.colored("{0: <23}".format("Name"), 'cyan') + ": " + termcolor.colored("{0}".format(k), 'yellow'))
                if 'module' in v:
                    print("    " + termcolor.colored("{0: <23}".format("Type"), 'cyan') + ": " + termcolor.colored("{0}".format(v['module']), 'yellow'))
                else:
                    print("    " + termcolor.colored("{0: <23}".format("Type"), 'cyan') + ": " + termcolor.colored("missing", 'red'))
                if 'overrides' in v:
                    print("    " + termcolor.colored("{0: <23}".format("Overrides"), 'cyan'))
                    for k2, v2 in v['overrides'].iteritems():
                        print("      " + termcolor.colored("{0: <21}".format(k2), 'cyan') + ": " + termcolor.colored("{0}".format(v2), 'yellow'))

    def spin(self):
        rospy.spin()
