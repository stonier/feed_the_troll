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

It also manages these for free. That is, it is able to bring the
dynamic reconfigure servers up and down in sync with the starting
up and tearing down of your higher level applications.

*Reconfigure your Reconfiguration!*

Server
======

You will need to prepare the following (no coding necessary!):

* A yaml defining the dyn-recfg servers you need
* A launcher for starting the reconfiguration server
* A launcher for starting the feeder with your yaml configuration

If you're familiar with nodelet managers and nodelets, this process is similar.
When the feeder node launches it sends a request to the server to fire up
the specified list of dynamic reconfigure servers. When it terminates,
it will shoot one last service call off to the reconfiguration server to
shutdown the previously started dynamic reconfigure servers.

Example - Launch Reconfiguration
================================

An example set of files (also available as a demo within this package):

.. literalinclude:: ../launch/demo_reconfiguration_server.launch
   :language: xml

feed it using this package's parameter feeder:

.. literalinclude:: ../launch/demo_reconfiguration_feeder.launch
   :language: xml

.. literalinclude:: ../parameters/demo_reconfiguration.yaml
   :language: yaml

A snapshot of the rosparam server is useful to illustrate where the various parameters
get sourced and eventually used for the reconfigure server. Note *dude* and *dudette* get
placed in two places - *dude* is inside the reconfiguration server, while *dudette* has been
explicitly instructed to start its dynamic reconfigure server elsewhere.

.. code-block:: bash

   $ rosparam list
   /feeder_snorriheim_24447_410607863113126015/parameters/dude/module
   /feeder_snorriheim_24447_410607863113126015/parameters/dude/overrides/bool_param
   /feeder_snorriheim_24447_410607863113126015/parameters/dude/overrides/double_param
   /feeder_snorriheim_24447_410607863113126015/parameters/dude/overrides/int_param
   /feeder_snorriheim_24447_410607863113126015/parameters/dude/overrides/size
   /feeder_snorriheim_24447_410607863113126015/parameters/dude/overrides/str_param
   /feeder_snorriheim_24447_410607863113126015/parameters/dudette/module
   /feeder_snorriheim_24447_410607863113126015/parameters/dudette/namespace
   /feeder_snorriheim_24447_410607863113126015/parameters/dudette/overrides/bool_param
   /feeder_snorriheim_24447_410607863113126015/parameters/dudette/overrides/double_param
   /feeder_snorriheim_24447_410607863113126015/parameters/dudette/overrides/int_param
   /feeder_snorriheim_24447_410607863113126015/parameters/dudette/overrides/size
   /feeder_snorriheim_24447_410607863113126015/parameters/dudette/overrides/str_param
   /feeder_snorriheim_24447_410607863113126015/server_namespace
   /foo/troll/dudette/bool_param
   /foo/troll/dudette/double_param
   /foo/troll/dudette/int_param
   /foo/troll/dudette/size
   /foo/troll/dudette/str_param
   /reconfiguration/debug
   /reconfiguration/dude/bool_param
   /reconfiguration/dude/double_param
   /reconfiguration/dude/int_param
   /reconfiguration/dude/size
   /reconfiguration/dude/str_param
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
