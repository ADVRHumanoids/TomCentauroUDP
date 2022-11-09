FROM arturolaurenzi/xbot2_focal_base_nvidia:latest

USER root
RUN apt-get install -y \ 
    ros-${rosdistro}-moveit-ros-planning \ 
    ros-${rosdistro}-moveit-ros-planning-interface 

SHELL ["/bin/bash", "-ic"]
USER user

RUN mkdir /home/user/tom_ws
WORKDIR /home/user/tom_ws

ENV HHCM_FOREST_CLONE_DEFAULT_PROTO=https

RUN forest init
RUN echo "source $PWD/setup.bash" >> /home/user/.bashrc
RUN echo 1 && forest add-recipes git@github.com:advrhumanoids/multidof_recipes.git --tag master 
RUN forest grow cartesio_collision_support -j4 --pwd user
RUN forest grow iit-centauro-ros-pkg -j4 --pwd user
RUN forest grow centauro_cartesio -j4
RUN forest grow tom_centauro_udp -j4
RUN forest grow iit-dagana-ros-pkg

