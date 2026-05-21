# sentinel_ros2
clone and build:

```bash
git clone https://github.com/Mingtao312/sentinel_ros2.git
cd sentinel_ros2
colcon build --symlink-install 
```


launch :

```bash
source install/setup.sh
ros2 launch sentinel_ros sentinel_launch.py 
```



record:


```bash
ros2 bag record /camera/left/rgb /camera/right/rgb /camera/left/diff /camera/right/diff /imu -o bagname
```

