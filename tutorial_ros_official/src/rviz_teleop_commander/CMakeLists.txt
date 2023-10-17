# catkin standard stuffs
cmake_minimum_required(VERSION 2.8.3)
project(rviz_teleop_commander)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS rviz)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# # QT related
# # # This plugin includes Qt widgets, so we must include Qt like so:
# find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
# include(${QT_USE_FILE})
# # # I prefer the Qt signals and slots to avoid defining "emit", "slots",
# # # etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
# add_definitions(-DQT_NO_KEYWORDS)
# # # Here we specify which header files need to be run through "moc",
# # # Qt's meta-object compiler.
# qt4_wrap_cpp(MOC_FILES
# src/teleop_pad.h
# )

# find qt5
set(Qt5QtCore_DIR /usr/lib/x86_64-linux-gnu/cmake)
find_package(Qt5 COMPONENTS Core Gui REQUIRED)
find_package(Qt5Widgets)

# set head with qt
qt5_wrap_cpp(TELEOP_PAD_HEAD src/teleop_pad.h)

# set src
set(TELEOP_PAD_SRC
    src/teleop_pad.cpp
    ${TELEOP_PAD_HEAD}
)

# add library
# # An rviz plugin is just a shared library, so here we declare the
# # library to be called ``${PROJECT_NAME}`` (which is
# # "rviz_plugin_tutorials", or whatever your version of this project
# # is called) and specify the list of source files we collected above
# # in ``${SOURCE_FILES}``.
add_library(${PROJECT_NAME} ${TELEOP_PAD_SRC})

# target_link_libraries
# # Link the library with whatever Qt libraries have been defined by
# # the ``find_package(Qt4 ...)`` line above, and with whatever libraries
# # catkin has included.
# # Although this puts "rviz_plugin_tutorials" (or whatever you have
# # called the project) as the name of the library, cmake knows it is a
# # library and names the actual file something like
# # "librviz_plugin_tutorials.so", or whatever is appropriate for your
# # particular OS.
# target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Widgets)

# install targets
install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install plugin description
install(FILES
    plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
