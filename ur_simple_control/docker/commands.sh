# some difficult to remember this here
# building. needs to be from root dir of repo
docker build -f docker/Dockerfile_for_apple_silicone -t smc .
# allowing opening windows
# macos
# install xquartz, enable networked clients or however it's called
# (details in installation)
# then
xhost +
docker run --rm -ti -e DISPLAY=host.docker.internal:0  -t smc /bin/bash
# on linux host
xhost +
# note: you don't need to share whole /tmp, just some files in it,
# but this is also good and shorter.
# same should work in windows wsl (maybe skip /tmp idk)
docker run --rm -ti -e DISPLAY=$DISPLAY -v /tmp:/tmp -t smc /bin/bash

# you can run your script directly instead of running bash
docker run --rm -ti -p 6666:6666 -p 7777:7777 -t slumi_translator python3 /bridging_scripts/protobuf_to_ros1.py
