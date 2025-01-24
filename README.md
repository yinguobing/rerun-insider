# rerun-insider
Log ROS messages to Rerun

## Configuration
Setup the subscription topic and the rerun viewer address in the config file.

- `subscription_topic_lidar` - Lidar topic
- `rerun_viewer_addr` - Rerun viewer address

Config file:`/path/to/install/rerun-insider/share/rerun-insider/config/insider.yaml`

## Run
```bash
ros2 launch rerun-insider insider.launch.py
```