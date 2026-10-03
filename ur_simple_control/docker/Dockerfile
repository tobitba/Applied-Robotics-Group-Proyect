#FROM ubuntu:jammy # required for ros
FROM ubuntu:noble

LABEL org.opencontainers.image.authors="marko.guberina@control.lth.se"

# NOTE: this is here just to install python3-tk without questions
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Stockholm

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3  \
        python3-pip \
        python3-tk \
        #ipython3 \
        git \
        sudo \
#        man-db \
#        manpages-posix \
        iputils-ping \
        arp-scan \
        # nice to have
#        python3-opencv \
        vim \
        vim-addon-manager \
        vim-youcompleteme \
        vim-python-jedi \
        zsh \
        zsh-syntax-highlighting \
        libarchive-tools \
        libxcb-cursor0 \
		python3-pyqt6 \
# opencv dependencies
		ffmpeg libsm6 libxext6 

#        python3-python-qt-binding \
#        robotpkg-py3*-pinocchio


# pinocchio install as debian package - compiled with numpy 1.26 so incompatible with pip which has numpy 2.2
# necessary for ubuntu 22
#RUN apt install -qqy lsb-release curl
#RUN mkdir -p /etc/apt/keyrings curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc
#RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list


# qt-binding is a really unnecessary 300MB, but i don't want
# to do more matplotlib hacks
# if you want manuals:
#RUN sed -i 's:^path-exclude=/usr/share/man:#path-exclude=/usr/share/man:' \
#        /etc/dpkg/dpkg.cfg.d/excludes
# --> we're running out of space on the docker.control
# and noone is reading manuals anyway
#RUN yes | unminimize 

# make the environment more usable
# create user - jammy comes with user ubuntu with UID 1000, so now we do 1001
RUN useradd -m -s /bin/zsh -G sudo -u 1001 student

WORKDIR /home/student/
RUN passwd -d student
USER student
# copy repo to workdir
RUN mkdir SimpleManipulatorControl
COPY --chown=student . ./SimpleManipulatorControl
RUN mkdir -p .cache/zsh/
COPY --chown=student /docker/dot_files_for_docker/.vimrc /home/student/
COPY --chown=student /docker/dot_files_for_docker/.zshrc /home/student/
COPY --chown=student /docker/dot_files_for_docker/global_extra_conf.py /home/student/


# this is enough to run clik
WORKDIR /home/student/
USER student
RUN pip install --break-system-packages -e ./SimpleManipulatorControl/python/
RUN pip install --break-system-packages pin crocoddyl matplotlib meshcat ur_rtde \
                qpsolvers ecos example_robot_data meshcat_shapes \
                pyqt6 opencv-python qpsolvers quadprog \
                proxsuite casadi pin-pink

# TODO: add optional instals here. (pyqt6 would be one of them. another is protobuf)

RUN vam install python-jedi && vam install youcompleteme
