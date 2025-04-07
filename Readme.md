# Urdf Visualization
---
## Build
### clean before rebuild
```bash
rm -r build
```
```bash
colcon build
```
### source env
```bash
source install/setup.bash
```
### launch rviz
```bash
ros2 launch x1_urdf display.launch.py urdf_path:=robot/x1_25dof/urdf/x1_25dof.urdf
```
default para(x1_29dof)
```bash
ros2 launch x1_urdf display.launch.py
```


## tool
```bash
check_urdf robot/x1_29dof/urdf/x1_ros.urdf

urdf_to_graphviz x1_ros.urdf | dot -Tpng -o structure.png
```

## another Vis Method
![urdf-loaders](https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/)
