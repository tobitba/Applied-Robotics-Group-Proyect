## Docker
----------
In the interest of time and convenience, the library is shipped as a Docker image.

### Installing docker on windows:
--------------------------------------
1. install wsl with "wsl --install" in powershell administrator, more at [windows docs](https://learn.microsoft.com/en-us/windows/wsl/install)
2. install docker on windows (with wsl option, should be default), more at [docker docs](https://docs.docker.com/desktop/install/windows-install)
3. create the docker accout 
4. open docker desktop, navigate to Settings (by user/account button) -> Resources -> Network -> click on enable host networking

### Installing docker on macos:
--------------------------------
1. go to [docker webpage](https://docs.docker.com/desktop/setup/install/mac-install/). 
   select the docker for your processor (intel or apple silicone).
   download and install docker-desktop.
2. if using user-level installation, add docker commands to path with "export PATH=[path_to_docker_bin,ex /Users/yourusername/.docker/bin]:$PATH".
   put this into your .bashrc or .zshrc if you don't want to do it every time you start the terminal (it's just another environment variable).
3. create the docker accout 
4. open docker desktop, navigate to Settings (by user/account button) -> Resources -> Network -> click on enable host networking
5. extra step to open windows from docker. install xquartz from xquartz.org. this is needed for real-time-plotting and other graphs to be shown
   (they need to open a window to do that). you also need to reboot after installing for the program to take effect.
   then you start the xquartz app. it won't open any window, but in the top-left corner you will see its name (next to apple logo).
   clik on that, go to settings -> security -> clik on "allow connections from network clients" (the docker container will use the localhost network 
   to interface). you need to reboot again for this to take effect.


### build or pull the image
------------------
Here are the steps to get to your container running once Docker is installed.
You can either build the image locally or pull it from our server (better because it won't take long).
To proceed you need to have the Docker desktop GUI app running (open).

#### building the image locally
--------------------------------
5. [WINDOWS] open wsl (type wsl in powershell) and navigate to git folder (now in wsl) with "cd /mnt/Users/YOURUSERNAME/PATH_TO_GIT_FOLDER"
5. [MAC] open a terminal and navigate to git folder with "cd /mnt/Users/YOURUSERNAME/PATH_TO_GIT_FOLDER"
6. build the image with "docker build -t smc -f docker/Dockerfile ." NOTE: if you are on mac and have an apple silicone processor,
    you might need to change the ubuntu version to one that works on the apple silicone processor.

#### pulling the image
-------------------------
5. login to our docker server with "docker login docker.control.lth.se -u docker_repo" with password "repo_Docker"
6. pull the image with "docker pull 

### using the image
----------------------
7. to run the image run  "docker run --rm -it --net=host -e DISPLAY=$DISPLAY -v /tmp:/tmp smc /bin/zsh".
   [Linux and MAC]: NOTE: FIRST RUN "xhost +" EVERY TIME before you run the image. otherwise docker can't open new windows,
   so real-time-plotting won't work. [MAC] you need to have xquartz installed to have the xhost + command (look at installation for more).
   NOTE! if you don't get real-time-plotting with xhost +, do xhost + 127.0.0.1 and replace -e DISPLAY=$DISPLAY with 
   -e DISPLAY=127.0.0.1:0 .
   [WINDOWS]: NOTE: i've been told that mapping the /tmp folder is not necessary on windows and that it in fact might cause problems.
   in this case just remove the "-v /tmp:/tmp" from the command
8. verify installation by running an example with --visualize-manipulator and --real-time-plotting arguments
9. if you want to make persistent changes to the code from the docker, you need to use the -v argument in docker run to share the folder
   with the code between your native OS (host) and the docker container. 
   in particular, use -v [path_to_folder_on_host]:[path_to_that_folder_in_docker_container]:rw .
   notice the :rw at the end. this allows for both the host and the container to modify the files (otherwise they will be read-only).

### Installing additional software on the image
-------------------------------------------------
Install things as you would normally (using apt in the terminal etc).
To get this to be a part of the Docker image, just use
the RUN [...] command in the Dockerfile, where you replace [...] with the
command you used to install what you installed normally.
Don't forget to build the image to apply these changes!
I highly recommend adding software this way, because
it will allow you to share the exact same set-up between your colleagues,
which will help you avoid headaches in the future.

NOTE that if you build many images docker will very quickly use up a lot of your hard drive. 
It does this because it saves the 
To solve this

## Native installation (installing ubuntu)
----------------------------------------------
1. Either create a disk partition for Ubuntu on your hard drive, or use an external hard drive. In the first case, you might need to shrink your existing partition. Searching for "how to create a disk partition [your_OS]" or "install ubuntu on [your_OS]" will get you all the information you need. Ideally, back up your data before any of this (you should be doing this in general as it's good practice).
2. Download an Ubuntu 22 LTH iso (file which contains everything needed for installation), available at official Ubuntu websites
3. Create a bootable USB. In short, you can't just copy an iso file to a USB and boot from it. Just google how to do it on your OS (use rufus if on windows).
4. Enter BIOS to ensure you can boot from your USB. Ideally this step should just be to enable Boot menu, but this step is dependent to a specific computer. Any reasonable Ubuntu (or any Linux) installation guide should have detailed steps, but also consult your laptop manufacturer websites for BIOS steps if the steps won't be obvious/easy to follow.
5. Boot from the Ubuntu USB and install it to the partition/external disk from step 1. MAKE SURE you are installing to the right partition. To select the partition, select the "Advanced installation" option (selecting the partition is the only advanced thing you need). Otherwise just follow the installation instructions you get once you boot into the USB, they are quite well done.

