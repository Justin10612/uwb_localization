# `uwb_localization`

## ROS2 Node:
- `uwb_receiver.py`:
- `localization.py`:
- `hyperbola_localization.py`:

## Python file:
- `uwb_init_test.py`
- `localization_test.py`

## 安裝:
**Run following script file**
```bash
cd robot_one
chmod +x clone_repos.sh
./clone_repos.sh
```

The package u need:
```bash
sudo apt-get install ros-foxy-xacro
sudo apt-get install ros-foxy-ros2-control
sudo apt-get install ros-foxy-ros2-controllers
sudo apt-get install ros-foxy-twist-mux
```
- For gazebo sim:
    ```bash
    sudo apt install ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-gazebo-ros2-control
    ```
## 使用方法
要啟動模擬，請執行以下命令：
```bash
ros2 launch robot_one launch_uwb_bot.launch.py
```
這將啟動 robot_one 套件，並使用 uwb_localization 套件進行模擬。
