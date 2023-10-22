Qt5-to-ROS2
===============

### Step by step solution to port Qt5 application project into compilabe ROS2 package.

1. Create a **cmake** Qt project. 
    - *All **qmake** projects should be be ported to cmake first ([CMake Port/Porting Guide](https://wiki.qt.io/CMake_Port/Porting_Guide))*.
2. Create a ROS2 workspace (e.g. *ros_qt_ws*) folder with `ros_qt_ws/src/` subdirectory
3. Move the Qt project to `ros_qt_ws/src/`.

```
ros_qt_ws/
└── src
    └── qt_demo
        ├── CMakeLists.txt
        ├── main.cpp
        ├── mainwindow.cpp
        ├── mainwindow.h
        └── mainwindow.ui
```

4. Establish ROS2 package hierarchy:
    - Move all the source files to a new directory: `ros_qt_ws/src/<Qt project>/src`
    - Move the header files to the `ros_qt_ws/src/<Qt project>/include/<Qt project>`
    - Currently the ROS2 workspace with the Qt project file tree should look similarly to this:

```
ros_qt_ws/
└── src
    └── qt_demo
        ├── CMakeLists.txt
        ├── include
        │   └── qt_demo
        │       └── mainwindow.h
        ├── package.xml
        └── src
            ├── main.cpp
            ├── mainwindow.cpp
            └── mainwindow.ui
```

5. Modify CMakeLists.txt file to comply with the [ament_cmake guidelines](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html). Example of such adjustment can be found in [ros_qt_ws/src/qt_demo/CMakeLists.txt](./ros_qt_ws/src/qt_demo/CMakeLists.txt)
    - A generic ROS2 package can be found under [test_pkg_ws/src/my_package](./test_pkg_ws/src/my_package). It has been generated as a quick reference using command: `ros2 pkg create --build-type ament_cmake --node-name my_node my_package`.

6. Create package.xml in `ros_qt_ws/src/<Qt project>/`

