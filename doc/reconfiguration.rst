.. _servers:

===============
Reconfiguration
===============

About
=====

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

It also manages these for free. You can start with an initial
collection of default dynamic reconfigure servers with their
respective configurations. You can then also dynamically
bring up, update or tear down servers as you wish at a later
time so that the reconfiguration syncs with the starting up
and tearing down of your higher level applications.

*Reconfigure your Reconfiguration!*

Server
======

You will need to prepare the following (no coding necessary!):

* A yaml defining the default dyn-recfg servers you need
* A launcher for starting the reconfiguration server
* (Optional) A yaml defining new dyn-recfg servers to manage, or update default ones
* (Optional) A launcher for starting the feeder with your yaml configuration

If you're familiar with nodelet managers and nodelets, the feeders work similarly.
When the feeder node launches it sends a request to the server to fire up
or update the specified list of dynamic reconfigure servers. When it terminates,
it will shoot one last service call off to the reconfiguration server to
shutdown or reset the previously started reconfigure servers.

Example - Launch Reconfiguration
================================

An example set of files (also available as a demo within this package):

.. literalinclude:: ../launch/demo_reconfiguration_server.launch
   :language: xml

.. literalinclude:: ../parameters/demo_reconfiguration_server.yaml
   :language: yaml

feed it using this package's parameter feeder:

.. literalinclude:: ../launch/demo_reconfiguration_feeder.launch
   :language: xml

.. literalinclude:: ../parameters/demo_reconfiguration_feeder.yaml
   :language: yaml

A snapshot of the rosparam server is useful to illustrate where the various parameters
get sourced and eventually used for the reconfigure server. Note *dude* and *dudette* get
placed in two places - *dude* is inside the reconfiguration server, while *dudette* has been
explicitly instructed to start its dynamic reconfigure server elsewhere.

.. code-block:: bash

   $ rosparam list
   /feeder_snorriwork_18767_3776865491990798862/parameters/dude/module
   /feeder_snorriwork_18767_3776865491990798862/parameters/dude/overrides/bool_param
   /feeder_snorriwork_18767_3776865491990798862/parameters/dude/overrides/double_param
   /feeder_snorriwork_18767_3776865491990798862/parameters/dude/overrides/int_param
   /feeder_snorriwork_18767_3776865491990798862/parameters/dude/overrides/size
   /feeder_snorriwork_18767_3776865491990798862/parameters/dude/overrides/str_param
   /feeder_snorriwork_18767_3776865491990798862/parameters/dudette/module
   /feeder_snorriwork_18767_3776865491990798862/parameters/dudette/namespace
   /feeder_snorriwork_18767_3776865491990798862/parameters/dudette/overrides/int_param
   /feeder_snorriwork_18767_3776865491990798862/server_namespace
   /foo/troll/dudette/bool_param
   /foo/troll/dudette/double_param
   /foo/troll/dudette/int_param
   /foo/troll/dudette/size
   /foo/troll/dudette/str_param
   /reconfiguration/bob/bool_param
   /reconfiguration/bob/double_param
   /reconfiguration/bob/int_param
   /reconfiguration/bob/size
   /reconfiguration/bob/str_param
   /reconfiguration/debug
   /reconfiguration/dude/bool_param
   /reconfiguration/dude/double_param
   /reconfiguration/dude/int_param
   /reconfiguration/dude/size
   /reconfiguration/dude/str_param
   /reconfiguration/servers/bob/module
   /reconfiguration/servers/dudette/module
   /reconfiguration/servers/dudette/namespace
   /reconfiguration/servers/dudette/overrides/bool_param
   /reconfiguration/servers/dudette/overrides/double_param
   /reconfiguration/servers/dudette/overrides/size
   /reconfiguration/servers/dudette/overrides/str_param
   /reconfiguration_client/name

Clients
=======

Client programs that need to tune into the dynamic reconfigure servers simply
need to instantiate a dynamic reconfigure client, or more simply, a subscriber
listening to the dynamic reconfigure server's private ``parameter_updates`` topic.

Example - Python Client
=======================

.. literalinclude:: ../scripts/demo_reconfiguration_client.py
   :language: python
   :lines: 10-27

Example - CPP Client
====================

There is no official C++ client, but it's easy to subscribe to the ``parameter_updates``
topic and use the ``config_->__fromMessage__`` method. Such a callback might look
similar to the following code:

.. code-block:: cpp

   typedef std::shared_ptr<feed_the_troll::DemoConfig> config_ptr DemoConfigPtr;

   void configCB(dynamic_reconfigure::ConfigPtr config_update_msg)
   {
      DemoConfigPtr config = DemoConfigPtr(new feed_the_troll::DemoConfig());
      config->__fromMessage__(*config_update_msg);
   }
