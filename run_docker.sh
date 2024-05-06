#!/bin/bash
docker run -ti --rm --network host -p 192.168.0.50:14561:14561/udp -p 192.168.0.50:14560:14560/udp -v "$PWD":/home/ilia/git --device=/dev/ttyACM0 --env=DISPLAY --name docker11_for_development gazebo11_22_04
