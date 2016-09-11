##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(feed_the_troll)

##############################################################################
# Find Packages
##############################################################################

find_package(catkin
    REQUIRED
    COMPONENTS
        dynamic_reconfigure
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()

##############################################################################
# Catkin
##############################################################################

catkin_python_setup()

generate_dynamic_reconfigure_options(
    dynamic_reconfigure/Demo.cfg
)

catkin_package(
    CATKIN_DEPENDS
        dynamic_reconfigure
)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS
        scripts/demo_param_server.py
        scripts/param_feeder.py
        scripts/reconfiguration_server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY dynamic_reconfigure DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch              DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY parameters          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})