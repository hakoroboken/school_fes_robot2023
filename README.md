# school_fes_robot2023
roboware program in school festival

# Roboware CI
このコードは[roboware CI](https://github.com/roboware-org/roboware-ci)によってテストされています。
|roboware_ci|[![build test](https://github.com/hakoroboken/school_fes_robot2023/actions/workflows/main.yml/badge.svg)](https://github.com/hakoroboken/school_fes_robot2023/actions/workflows/main.yml)|
|:--:|:--:|

# Usage
```
git clone https://github.com/hakoroboken/school_fes_robot2023.git
```
```
cd school_fes_robot2023
```
```
colcon buld
source ./install/setup.bash
```
### wifi
```
ros2 launch roboware_launch manual_b.launch.xml
```
### no wifi
```
ros2 launch roboware_launch no_wifi.launch.xml
```
