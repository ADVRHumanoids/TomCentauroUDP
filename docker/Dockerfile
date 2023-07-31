FROM arturolaurenzi/xbot2_focal_base_nvidia:latest

USER root
RUN apt-get update && apt-get install -y \ 
    ros-${rosdistro}-moveit-ros-planning \ 
    ros-${rosdistro}-moveit-ros-planning-interface 

SHELL ["/bin/bash", "-ic"]
USER user

RUN mkdir /home/user/tom_ws
WORKDIR /home/user/tom_ws

ENV HHCM_FOREST_CLONE_DEFAULT_PROTO=https

RUN forest init
RUN echo "source $PWD/setup.bash" >> /home/user/.bashrc

WORKDIR /home/user/tom_ws/src 
RUN echo 1 && git clone https://github.com/advrhumanoids/tomcentauroudp.git --branch dual_arm_2023 tom_centauro_udp

WORKDIR /home/user/tom_ws/ros_src 
RUN git clone https://github.com/advrhumanoids/iit-inail2arm-ros-pkg.git --branch dual_arm

WORKDIR /home/user/tom_ws
RUN forest add-recipes git@github.com:advrhumanoids/multidof_recipes.git --tag master 
RUN forest grow tom_centauro_udp -j4
RUN forest grow iit-dagana-ros-pkg -j4

