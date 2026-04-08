#usr/bin/env bash

ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "guided"}'
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL '{altitude: 3}'
