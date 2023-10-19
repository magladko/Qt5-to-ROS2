Qt5-to-ROS2
===============

### Step by step solution to move Qt5 application project into compilabe ROS2 package.

1. Create Qt project with cmake or qmake.
2. Create a ROS2 workspace folder with `<ws>/src/` subdirectory
3. Move the Qt project to `<ws>/src/`.

```
└── ros_qt_ws
    └── src
        └── qt_demo
            ├── CMakeLists.txt
            ├── CMakeLists.txt.user
            ├── main.cpp
            ├── mainwindow.cpp
            ├── mainwindow.h
            └── mainwindow.ui
```


4. Establish ROS2 package hierarchy:
    - Move all the files besides CMakeLists.txt to new directory: `<ws>/src/<Qt project>/src`
    - Currently the ROS2 workspace with the Qt project file tree should look similarly to this:

```
└── ros_qt_ws
    └── src
        └── qt_demo
            ├── CMakeLists.txt
            ├── CMakeLists.txt.user
            └── src
                ├── main.cpp
                ├── mainwindow.cpp
                ├── mainwindow.h
                └── mainwindow.ui
```

5. Modify CMakeLists.txt file to comply with the [ament_cmake guidelines](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html)

6. Create package.xml in `<ws>/src/<Qt project>/`
