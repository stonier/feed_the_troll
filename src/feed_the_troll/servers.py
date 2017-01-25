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
import functools
import importlib
import rosgraph
import rospy
import termcolor
import threading

##############################################################################
# helpers
##############################################################################


def validate_server_parameters(server_parameters):
    """
    Take in a reconfiguration setup on the parameter server (dic of dyn reconf
    server names with corresponding namespace, module & overrides values) and
    check if it is in a proper format as well as whether the module too exists.
    """
    error_messages = []
    for v in server_parameters.values():
        if 'module' not in v:
            error_messages.append("no dynamic reconfigure 'module' specified in the reconfigure server settings (e.g. 'feed_the_troll.cfg.DemoConfig']")
            continue
        try:
            importlib.import_module(v['module'])
        except ImportError:
            error_messages.append("could not import dynamic reconfigure module [{0}]".format(v['module']))
            continue
    return error_messages


def namespace_from_configuration(server_name, server_configuration):
    """
    :param ... configuration: troll reconfiguration server parameters (name, namespace, overrides)
    """
    namespace = server_configuration['namespace'] if 'namespace' in server_configuration else server_name
    if not namespace.startswith('/') and not namespace.startswith('~'):
        namespace = rosgraph.names.ns_join('~', namespace)
    return rospy.resolve_name(namespace)

##############################################################################
# ReConfiguration
##############################################################################


class ReconfigureServerInfo(object):
    """
    A simple class holding current information about a running dynamic
    reconfigure server.
     """
    def __init__(self):
        self.is_default_instance = False  # is the current instance a default server?
        self.default_configuration = None  # if there is a default server, what is it's configuration?
        self.namespace = None  # where the parameters are stored (dyn reconf server doesnt actuall save this, so we do
        self.server = None  # the currently running instance


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
        self.guard = threading.Lock()
        self.reconfigure_servers = {}
        self.debug = rospy.get_param("~debug", False)
        self._start_default_servers()
        # must come after everything else is set up
        self.troll = feed_the_troll.trolls.ROSParameters(
            loading_handler=self._load,
            unloading_handler=self._unload
        )

    def _start_server(self, server_name, server_configuration):
        """
        :param str name: unique name string for the server
        :param ... configuration: troll reconfiguration server parameters (name, namespace, overrides)
        """
        reconfigure_module = importlib.import_module(server_configuration['module'])
        reconfigure_server_namespace = namespace_from_configuration(server_name, server_configuration)

        default_config = reconfigure_module.defaults.copy()
        for parameter in default_config:
            full_parameter_path = reconfigure_server_namespace + "/" + parameter
            rospy.set_param(full_parameter_path, default_config[parameter])

        if 'overrides' in server_configuration:
            parameters = {}
            namespace_exists = rospy.has_param(reconfigure_server_namespace)
            if namespace_exists:
                parameters = rospy.get_param(reconfigure_server_namespace)
            parameters.update(server_configuration['overrides'])
            rospy.set_param(reconfigure_server_namespace, parameters)

        return dynamic_reconfigure.server.Server(
            reconfigure_module,
            functools.partial(self.callback, name=server_name),
            namespace=reconfigure_server_namespace
        )

    def _start_default_servers(self):
        try:
            default_server_parameters = rospy.get_param("~servers")
        except KeyError:
            return  # nothing to do
        error_messages = validate_server_parameters(default_server_parameters)
        if error_messages:
            rospy.logerr("Reconfiguration: errors in the default server configurations")
            for message in error_messages:
                rospy.logerr("               : {0}".format(message))
            return
        for server_name, server_configuration in default_server_parameters.iteritems():
            self.reconfigure_servers[server_name] = ReconfigureServerInfo()
            self.reconfigure_servers[server_name].is_default_instance = True
            self.reconfigure_servers[server_name].default_configuration = server_configuration
            self.reconfigure_servers[server_name].namespace = namespace_from_configuration(server_name, server_configuration)
            self.reconfigure_servers[server_name].server = self._start_server(server_name, server_configuration)

    def _load(self, unique_identifier, namespace):
        """
        :param uuid.UUID unique_identifier:
        :param str namespace: root namespace for configuration on the parameter server
        """
        try:
            incoming_configuration = rospy.get_param(namespace)
        except KeyError:
            error_message = "could not retrieve parameters for configuration [{0}]".format(namespace)
            rospy.logerr("Reconfiguration: {0}".format(error_message))
            return (False, error_message)
        self._pretty_print_incoming("Reconfigure Loading", unique_identifier, namespace, incoming_configuration)
        # checks
        error_messages = validate_server_parameters(incoming_configuration)
        with self.guard:
            for k in incoming_configuration.keys():
                if k in self.reconfigure_servers and not self.reconfigure_servers[k].is_default_instance:
                    error_messages.append("this reconfigure server is already being served [{0}]".format(k))
                    continue
        if error_messages:
            rospy.logerr("Reconfiguration: errors loading the passed parameterisations")
            for message in error_messages:
                rospy.logerr("               : {0}".format(message))
            return (False, ', '.join(error_messages))
        # setup
        with self.guard:
            for server_name, server_configuration in incoming_configuration.iteritems():
                if server_name not in self.reconfigure_servers:
                    self.reconfigure_servers[server_name] = ReconfigureServerInfo()
                    self.reconfigure_servers[server_name].is_default_instance = False
                    self.reconfigure_servers[server_name].default_configuration = None
                    self.reconfigure_servers[server_name].namespace = namespace_from_configuration(server_name, server_configuration)
                    self.reconfigure_servers[server_name].server = self._start_server(server_name, server_configuration)
                else:
                    # at this point, we know it is a deafult instance (we reject running and non-default above)
                    #   1. save the latest default configuration
                    current_dynamic_reconfigure_parameters = rospy.get_param(self.reconfigure_servers[server_name].namespace)
                    self.reconfigure_servers[server_name].default_configuration['overrides'] = current_dynamic_reconfigure_parameters
                    #   2. set the new parameters
                    # magically merge current and incoming (this only works when keys are strings - http://treyhunner.com/2016/02/how-to-merge-dictionaries-in-python/
                    new_parameters = dict(current_dynamic_reconfigure_parameters, **incoming_configuration[server_name]['overrides'])
                    self.reconfigure_servers[server_name].server.update_configuration(new_parameters)
                    #   3. set is_default_instance to False
                    self.reconfigure_servers[server_name].is_default_instance = False
        return (True, "Success")

    def _unload(self, unique_identifier, namespace):
        """
        :param uuid.UUID unique_identifier:
        """
        error_messages = []
        parameters = rospy.get_param(namespace)
        self._pretty_print_incoming("Reconfigure Unloading", unique_identifier, namespace, parameters)
        with self.guard:
            for server_name, unused_v in parameters.iteritems():
                reflected_parameters = rosgraph.names.ns_join("~", server_name)
                if rospy.has_param(reflected_parameters):
                    rospy.delete_param(reflected_parameters)
                if server_name not in self.reconfigure_servers:
                    error_messages.append("could not find server to unload [{0}]".format(server_name))
                    continue
                if self.reconfigure_servers[server_name].is_default_instance:
                    error_messages.append("refusing to unload a default instance [{0}]".format(server_name))
                    continue
                server_info = self.reconfigure_servers[server_name]
                if server_info.default_configuration is None:  # its a server created on the fly
                    server_info.server.set_service.shutdown()
                    server_info.server.descr_topic.unregister()
                    server_info.server.update_topic.unregister()
                    del server_info.server.set_service
                    del server_info.server
                    self.reconfigure_servers.pop(server_name)
                else:  # there is a default instance and configuration behind it
                    server_info.server.update_configuration(server_info.default_configuration['overrides'])
                    server_info.is_default_instance = True
        if error_messages:
            rospy.logerr("Reconfiguration: errors while unloading")
            for message in error_messages:
                rospy.logerr("               : {0}".format(message))
            return (False, ', '.join(error_messages))
        else:
            return (True, "Success")

    def callback(self, config, level, name):
        """
        The name is an additional argument not usually in a dynamic reconfigure server callback, but is fixed by
        a functools partial so that we can provide extra useful debugging information by stating *which* dynamic
        reconfigure server it relates to.

        :param dynamic_reconfigure.encoding.Config config: dynamic reconfigure configuration object, holds all the variables
        :param int level:
        :param str name: name of the reconfiguration server for which these configuration variables apply
        """
        if self.debug:
            print("")
            termcolor.cprint("Reconfiguration Updating", 'white', attrs=['bold'])
            print("")
            termcolor.cprint("  Reconfigure Server", "green")
            print("    " + termcolor.colored("{0: <23}".format("Name"), 'cyan') + ": " + termcolor.colored("{0}".format(name), 'yellow'))
            print("    " + termcolor.colored("{0: <23}".format("Namespace"), 'cyan') + ": " + termcolor.colored("{0}".format(self.reconfigure_servers[name].namespace), 'yellow'))
            termcolor.cprint("    Overrides", "cyan")
            for k, v in config.iteritems():
                if k != "groups":
                    print("      " + termcolor.colored("{0: <21}".format(k), 'cyan') + ": " + termcolor.colored("{0}".format(v), 'yellow'))
        return config

    def _pretty_print_incoming(self, title, unique_identifier, namespace, parameters):
        if self.debug:
            print("")
            termcolor.cprint(title, 'white', attrs=['bold'])
            print("")
            print("  " + termcolor.colored("{0: <25}".format('Feeder'), 'green'))
            print("    " + termcolor.colored("{0: <23}".format('Namespace'), 'cyan') + ": " + termcolor.colored("{0}".format(namespace), 'yellow'))
            print("    " + termcolor.colored("{0: <23}".format('Unique Identifier'), 'cyan') + ": " + termcolor.colored("{0}".format(unique_identifier), 'yellow'))
            for k, v in parameters.iteritems():
                print("  " + termcolor.colored("{0}".format("Reconfigure Server"), 'green'))
                print("    " + termcolor.colored("{0: <23}".format("Name"), 'cyan') + ": " + termcolor.colored("{0}".format(k), 'yellow'))
                print("    " + termcolor.colored("{0: <23}".format("Namespace"), 'cyan') + ": " + termcolor.colored("{0}".format(namespace_from_configuration(k, v)), 'yellow'))
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
