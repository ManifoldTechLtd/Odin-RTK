#!/bin/bash
sudo chmod 777 /dev/ttyACM0
python3 /home/xxx/ws_odin_rtk/src/rtk_fusion/rtk_string_publisher_ros2.py
