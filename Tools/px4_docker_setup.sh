docker run -it --privileged \
--env=LOCAL_USER_ID="$(id -u)" \
-v $(pwd):/workspaces/omnicopter:rw \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e DISPLAY=$DISPLAY \
-p 18570:18570/udp \
--name=PX4_ROS_container px4io/px4-dev-ros-noetic:2021-09-08 bash
