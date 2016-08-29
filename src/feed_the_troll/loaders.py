#!/usr/bin/env python
#
# License: MIT
#   https://raw.github.com/stonier/feed_the_troll/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: loaders
   :platform: Unix
   :synopsis: Various loaders
"""

##############################################################################
# Imports
##############################################################################

import feed_the_troll_msgs.srv as feed_the_troll_srvs
import rosgraph
import rospy
import unique_id

##############################################################################
# Classes
##############################################################################


class Base(object):
    def __init__(self, service_namespace="~"):
        """
        @param str service_namespace:
        """
        self._service_namespace = rospy.resolve_name(service_namespace)  # converts relateive, '~' -> global appropriately
        self._unload_service_name = rosgraph.names.ns_join(self._service_namespace, 'unload')
        self._unload_configuration = rospy.ServiceProxy(self._unload_service_name, feed_the_troll_srvs.Unload)
        self._unique_identifier = unique_id.toMsg(unique_id.fromRandom())


class ParamServerFeeder(Base):
    """
    The generic feeder script should lookup parameters, enable remappings to handle
    setting of the constructor arguments. This interface is a programming interface
    only.

    .. warning:: this blocks indefinitely if it can't find the configuration server

    .. todo:: a means of injecting a timeout by a user

    .. todo:: bond construction/destruction on loading/unloading, just like nodelets do
    """
    def __init__(self, service_namespace="~", configuration_namespace="~parameters"):
        """
        The service namespace should be cast at the root of where the troll's services
        can be found (load/param || load/yaml, unload). This is a means for programmatically
        handling the connections. You can of course, also just remap in a roslaunch.

        @param str service_namespace: where to find the troll services for loading/unloading
        @param str configuration_namepsace:
        """
        super(ParamServerFeeder, self).__init__(service_namespace)

        #############################
        # Parameterisation
        #############################
        self._configuration_namespace = rospy.resolve_name(configuration_namespace)

        load_service_name = rosgraph.names.ns_join(rosgraph.names.ns_join(self._service_namespace, 'load'), 'param')
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service(load_service_name, timeout=3.0)
                break
            except rospy.ROSException:  # timeout
                rospy.logwarn("Troll Feeder: cannot find the troll to load configuration [{0}][{1}]".format(self._configuration_namespace,
                                                                                                            load_service_name))
                continue
            except rospy.ROSInterruptException:
                # may not actually reach here because time, subsequently timeouts may be awry, so check rospy.is_shutdown always!
                return
        if rospy.is_shutdown():
            return
        try:
            load_configuration = rospy.ServiceProxy(load_service_name, feed_the_troll_srvs.LoadFromRosParam)
            response = load_configuration(unique_identifier=self._unique_identifier,
                                          namespace=self._configuration_namespace)
            if not response.result:
                rospy.logerr("Troll Feeder: failed to load configuration [{0}]".format(response.message))
        except rospy.ServiceException as e:
            rospy.logerr("Troll Feeder: failed to contact the configuration server [{0}][{1}]".format(load_service_name, str(e)))

        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        try:
            response = self._unload_configuration(unique_identifier=self._unique_identifier)
            if not response.result:
                rospy.logerr("Troll Feeder: failed to unload configuration [{0}]".format(response.message))
        except rospy.ServiceException as e:
            rospy.logerr("Troll Feeder: failed to contact the configuration server [{0}][{1}]".format(self._unload_service_name, str(e)))
