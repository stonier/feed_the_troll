#!/usr/bin/env python
#
# License: MIT
#   https://raw.github.com/stonier/feed_the_troll/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: trolls
   :platform: Unix
   :synopsis: Trolls dynamically digest configuration at runtime.
"""

##############################################################################
# Imports
##############################################################################

import feed_the_troll_msgs.srv as feed_the_troll_srvs
import rocon_console.console as console
import rosgraph
import rospy
import unique_id

##############################################################################
# Classes
##############################################################################


class Base(object):
    def __init__(self,
                 loading_handler=None,
                 unloading_handler=None,
                 service_namespace="~",
                 ):
        """
        :param function loading_handler: with signature (uuid_msgs/UniqueID unique_identifier, str param_server_namespace), returns (bool, message)
        :param function unloading_handler: with signature (uuid_msgs/UniqueID unique_identifier), returns (bool, message)
        :param str service_namespace:
        """
        self._loading_handler = loading_handler
        self._unloading_handler = unloading_handler
        self._service_namespace = rospy.resolve_name(service_namespace)  # converts relative, '~' to global appropriately
        self._unload_service = rospy.Service(rosgraph.names.ns_join(self._service_namespace, 'unload'), feed_the_troll_srvs.Unload, self.unload)
        self.loaded = {}
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        """
        Cleanup...
        """
        pass
        # call the unload handler?

    def load(self, unique_identifier, configuration):
        """
        Common loading actions required by the various configuration servers.

        :param uuid_msgs/UniqueID unique_identifier:
        :param configuration: arbitrary value decided upon by children (e.g. ros param namespace for param configuration)
        """
        # @todo check if this has already been loaded - use policies to decide how to handle?
        self.loaded[unique_id.fromMsg(unique_identifier)] = configuration

    def unload(self, request):
        """
        Handle a configuration unload request from a feeder.

        :param feed_the_troll_msgs.UnloadRequest request:
        """
        response = feed_the_troll_srvs.UnloadResponse(result=0,
                                                      message="no handler, so configuration not unloaded"
                                                      )

        uuid_identifier = unique_id.fromMsg(request.unique_identifier)
        if self._unloading_handler is not None:
            (response.result, response.message) = self._unloading_handler(uuid_identifier, self.loaded[uuid_identifier])
        try:
            self.loaded.pop(uuid_identifier)
            # let the user decide what/what not to log via the unloading handler
            # rospy.loginfo("Troll: unloaded configuration [{0}][{1}]".format(unique_id.fromMsg(request.unique_identifier), self._service_namespace))
        except KeyError:
            rospy.logwarn("Troll: trying to unload an unknown configuration [{0}][{1}]".format(unique_id.fromMsg(request.unique_identifier)), self._service_namespace)

        return response


class ROSParamConfiguration(Base):
    """
    ...
    """
    def __init__(self,
                 loading_handler=None,
                 unloading_handler=None,
                 service_namespace="~",
                 ):
        """
        :param str service_namespace:
        :param str handler:
        """
        super(ROSParamConfiguration, self).__init__(loading_handler, unloading_handler, service_namespace)
        self._load_service = rospy.Service(self._service_namespace + 'load/param', feed_the_troll_srvs.LoadFromRosParam, self.load)

    def load(self, request):
        """
        :param feed_the_troll_msgs.LoadFromParamServerRequest request:
        """
        response = feed_the_troll_srvs.LoadFromRosParamResponse(
            result=0,
            message="no handler, so configuration not processed"
        )
        super(ROSParamConfiguration, self).load(request.unique_identifier, request.namespace)

        if self._loading_handler is not None:
            (response.result, response.message) = self._loading_handler(unique_id.fromMsg(request.unique_identifier), request.namespace)
        return response

    def __str__(self):
        s = ""
        if len(self.loaded.keys()) > 0:
            s += console.green + "Loaded Configurations\n"
        for key, value in self.loaded.iteritems():
            s += "  " + console.cyan + "{0}".format(key) + console.reset + ": " + console.yellow + "{0}\n".format(value) + console.reset
        return s


# class YamlConfiguration(Base):
#     """
#     ...
#     """
#     def __init__(self,
#                  loading_handler=None,
#                  unloading_handler=None,
#                  service_namespace="~",
#                  ):
#         """
#         :param str service_namespace:
#         :param str handler:
#         """
#         super(YamlStringConfiguration, self).__init__(loading_handler, unloading_handler, service_namespace)
#         self.unload_service = rospy.Service(self._service_namespace + 'load/yaml', feed_the_troll_srvs.LoadFromYamlString, self.digest)
#
#     def digest(self, request):
#         """
#         :param feed_the_troll_msgs.LoadFromYamlStringRequest request:
#         """
#         response = feed_the_troll_srvs.LoadFromYamlStringResponse(result=0,
#                                                                   message="no handler, so configuration not processed"
#                                                                   )
#         if self._yaml_loading_handler is not None:
#             (response.result, response.message) = self._yaml_loading_handler(request.unique_identifer,
#                                                                              request.yaml)
#         return response
#
