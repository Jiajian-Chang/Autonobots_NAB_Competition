#!/bin/bash
ros2 topic pub /e_stop --once  std_msgs/msg/Bool "{data: true}"

