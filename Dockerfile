FROM ros:noetic

USER root

SHELL ["/bin/bash", "-c"]

# Avoid interactive (e.g. select geographic region)
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get upgrade -y
RUN apt-get install lsb-core -y
RUN apt-get install git -y
RUN apt-get install python3-pip -y
RUN apt-get install vim -y
RUN apt-get install gnome-terminal -y
RUN apt-get install libcanberra-gtk-module libcanberra-gtk3-module dbus-x11 -y
RUN apt-get install wget -y
RUN apt-get install iputils-ping -y
RUN apt-get install x11-xserver-utils -y

# Install libgfortran3
RUN cd /tmp/ && wget http://archive.ubuntu.com/ubuntu/pool/universe/g/gcc-6/gcc-6-base_6.4.0-17ubuntu1_amd64.deb
RUN cd /tmp/ && wget http://archive.ubuntu.com/ubuntu/pool/universe/g/gcc-6/libgfortran3_6.4.0-17ubuntu1_amd64.deb
RUN cd /tmp/ && dpkg -i gcc-6-base_6.4.0-17ubuntu1_amd64.deb
RUN cd /tmp/ && dpkg -i libgfortran3_6.4.0-17ubuntu1_amd64.deb
RUN rm -rf /tmp/*

# Install python packages
RUN apt-get install make
RUN python3 -m pip install numpy --upgrade scipy rockit-meco odroid-wiringpi multipledispatch
RUN python3 -m pip --no-cache-dir install sphinx-gallery sphinx_rtd_theme sphinx-markdown-parser sphinx_mdinclude myst-parser

# Install extra ROS tools
RUN apt-get install ros-noetic-rviz -y
RUN apt-get install ros-noetic-map-server -y
RUN apt-get install python3-catkin-tools -y
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Copy code from this repository, build catkin workspaces
COPY . /truck-trailer-amr
RUN source /opt/ros/noetic/setup.bash && cd /truck-trailer-amr/catkin_ws_offboard && catkin clean -y && catkin build
# RUN cd /home/docker/truck-trailer-amr/catkin_ws_onboard && catkin clean -y && catkin build

# Library path include hsl binaries
RUN echo 'export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/truck-trailer-amr/hsl/"' >> ~/.bashrc 

# When running the container, enter in this directory
WORKDIR /truck-trailer-amr

# Make networking with ROS available
EXPOSE 11011-11310
EXPOSE 11311

CMD /bin/bash -c "cd /truck-trailer-amr/execution/ && source run_sim; exec -i bash"
# CMD ["/bin/bash", "--norc"]
