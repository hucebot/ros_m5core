docker run -it --rm -v `pwd`:/project -v /dev:/dev --volume=/etc/timezone:/etc/timezone:ro --privileged --net host --workdir /project ros_m5core:latest
