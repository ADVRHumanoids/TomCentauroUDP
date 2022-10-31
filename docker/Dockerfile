FROM arturolaurenzi/xbot2_focal_base_nvidia:latest
USER user
SHELL ["/bin/bash", "-ic"]

RUN mkdir /home/user/tom_ws
WORKDIR /home/user/tom_ws

ENV HHCM_FOREST_CLONE_DEFAULT_PROTO=https

RUN forest init
RUN echo "source $PWD/setup.bash" >> /home/user/.bashrc
RUN forest add-recipes git@github.com:advrhumanoids/multidof_recipes.git --tag master 
RUN echo 0 && forest grow iit-centauro-ros-pkg -j4 --pwd user
RUN echo 1 && forest grow tom_centauro_udp -j4

