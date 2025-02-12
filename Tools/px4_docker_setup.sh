docker run -it --privileged \
--env=LOCAL_USER_ID="$(id -u)" \
-v /home/patrick/quadcopter-lqr-controller:/quadcopter-lqr-controller/:rw \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e DISPLAY=$DISPLAY \
-p 14570:14570/udp \
--name=PX4_ROS_container px4io/px4-dev-ros-noetic:latest bash
