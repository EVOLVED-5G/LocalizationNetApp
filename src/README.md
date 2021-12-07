# LocalizationNetApp

To build the app you must have installed ros2 on your machine and build the package

```shell
  cd LocalizationNetApp/
  colcon build --symlink-install
  . install/setup.bash
```

Than the test node can be launched using

```shell
ros2 run localization_netapp test_node
```

