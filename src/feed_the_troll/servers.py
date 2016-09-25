#
# License: MIT
#   https://raw.githubusercontent.com/stonier/feed_the_troll/devel/LICENSE
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
    **About**

    *Q) What to do when you need to share dynamic_reconfigure variables
    amongst a collection of nodes?*

    A good example is sharing navigation motion constraints amongst
    a collection of nodes that control the motion of the base (general
    navigation, parking, docking, ...)

    This standalone node volunteers for the responsibility of loading and
    managing dynamic reconfigure servers from a central location. All you
    need do is feed it with a yaml/rosparam configuration that will define
    the dynamic_reconfigure servers you wish to fire up along with any
    initial parameter overrides to use when instantiating them.

    It also manages these for free. That is, it is able to bring the
    dynamic reconfigure servers up and down in sync with the starting
    up and tearing down of your higher level applications.

    *Reconfigure your Reconfiguration!*

    **Usage - Reconfiguration Server**

    You will need to prepare the following (no coding necessary!):

    * A yaml defining the dyn-recfg servers you need
    * A launcher for starting the reconfiguration server
    * A launcher for starting the feeder with your yaml configuration

    If you're familiar with nodelet managers and nodelets, this process is similar.
    When the feeder node launches it sends a request to the server to fire up
    the specified list of dynamic reconfigure servers. When it terminates,
    it will shoot one last service call off to the reconfiguration server to
    shutdown the previously started dynamic reconfigure servers.

    **Example - Reconfiguration Server**

    An example set of files (also available as a demo within this package):

    .. code-block:: xml

    .. literalinclude:: ../launch/demo_reconfiguration_server.launch
       :language: xml

    feed it using this package's parameter feeder:

    .. literalinclude:: ../launch/demo_reconfiguration_feeder.launch
       :language: xml

    .. literalinclude:: ../parameters/demo_reconfiguration.yaml
       :language: yaml

    **Usage - Reconfiguration Clients**

    Client programs that need to tune into the dynamic reconfigure servers simply
    need to instantiate a dynamic reconfigure client, or more simply, a subscriber
    listening to the dynamic reconfigure server's private ``parameter_updates`` topic.

    **Examples - Reconfiguration Clients**

    Python:

    .. literalinclude:: ../scripts/demo_reconfiguration_client.py
       :language: python
       :lines: 10-27

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
                reconfigure_server_namespace = v['namespace'] if 'namespace' in v else '~'
                if 'overrides' in v:
                    rospy.set_param(rosgraph.names.ns_join(reconfigure_server_namespace, k), v['overrides'])
                self.reconfigure_servers[k] = dynamic_reconfigure.server.Server(
                    reconfigure_module,
                    self.callback,
                    namespace=rosgraph.names.ns_join(reconfigure_server_namespace, k)
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
                if k != "groups":
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
