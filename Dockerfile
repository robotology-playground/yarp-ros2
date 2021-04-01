FROM randaz81/r1slam:ros2
LABEL maintainer="daniele.domenichelli@iit.it"

ARG username
ARG uid
ARG gid

USER root

# Fix the user
RUN groupmod -g $gid $username && \
    usermod -u $uid -g $gid -G adm,users,sudo,root $username && \
    find / -uid 33334 -exec chown -h $uid '{}' \; 2> /dev/null || true && \
    find / -gid 33334 -exec chgrp $gid '{}' \; 2> /dev/null || true && \
    chown -R $username: /home/$username && \
    mkdir -p /run/user/$uid && \
    chown $username: /run/user/$uid && \
    chmod 700 /run/user/$uid && \
    mkdir -p /run/user/$uid/dconf && \
    chown $username: /run/user/$uid/dconf && \
    chmod 700 /run/user/$uid/dconf && \
    mkdir -p /home/$username/.ros && \
    chown -R $username: /home/$username/.ros

# RUN cd /home/$username/ycm/build && \
#     make uninstall && \
#     cd .. && \
#     git fetch --all --prune && \
#     git reset --hard origin/master && \
#     cd build && \
#     cmake . && \
#     make -j4 && \
#     make install && \
#     chown -R $username: /home/$username/ycm


# RUN cd /home/$username/yarp/build && \
#     make uninstall && \
#     cd .. && \
#     git fetch --all --prune && \
#     git reset --hard origin/master && \
#     cd build && \
#     cmake . && \
#     make -j4 && \
#     make install && \
#     chown -R $username: /home/$username/yarp


# RUN cd /home/$username/icub-main/build && \
#     make uninstall && \
#     cd .. && \
#     git fetch --all --prune && \
#     git reset --hard origin/devel && \
#     cd build && \
#     cmake . && \
#     make -j4 && \
#     make install && \
#     chown -R $username: /home/$username/icub-main


RUN cd /home/$username/gazebo-yarp-plugins/build && \
    make uninstall && \
    cd .. && \
    git remote add drdanz https://github.com/drdanz/gazebo-yarp-plugins.git && \
    git fetch --all --prune && \
    git checkout -b wip_cbw drdanz/wip_cbw && \
    cd build && \
    cmake . && \
    make -j4 && \
    make install && \
    chown -R $username: /home/$username/gazebo-yarp-plugins
#     git reset --hard origin/devel && \

# RUN cd /home/$username/navigation/build && \
#     make uninstall && \
#     cd .. && \
#     git fetch --all --prune && \
#     git reset --hard origin/master && \
#     cd build && \
#     cmake . && \
#     make -j4 && \
#     make install && \
#     chown -R $username: /home/$username/navigation


# RUN cd /home/$username/idyntree/build && \
#     make uninstall && \
#     cd .. && \
#     git fetch --all --prune && \
#     git reset --hard origin/devel && \
#     cd build && \
#     cmake . && \
#     make -j4 && \
#     make install && \
#     chown -R $username: /home/$username/idyntree


RUN cd /home/$username/cer-sim && \
    git remote add randaz81 https://github.com/randaz81/cer-sim.git && \
    git fetch --all --prune && \
    git checkout -b ros2nws randaz81/ros2nws && \
    cd /home/$username/cer-sim/catkin_ws/src && \
    bash -c " \
        source /opt/ros/noetic/setup.bash && \
        catkin_init_workspace" && \
    cd /home/$username/cer-sim/catkin_ws && \
    bash -c " \
        source /opt/ros/noetic/setup.bash && \
        catkin_make" && \
    cd /home/$username/cer-sim/colcon_ws && \
    bash -c " \
        source /opt/ros/foxy/setup.bash && \
        colcon build" && \
    chown -R $username: /home/$username/cer-sim


COPY . /home/$username/yarp-ros2
RUN cd /home/$username/yarp-ros2/ros2_interfaces_ws && \
    bash -c " \
        source /opt/ros/foxy/setup.bash && \
        colcon build --packages-select map2d_nws_ros2_msgs" && \
    cd /home/$username/yarp-ros2 && \
    mkdir build && \
    cd build && \
    bash -c " \
        source /opt/ros/foxy/setup.bash && \
        source /home/$username/yarp-ros2/ros2_interfaces_ws/install/setup.bash && \
        cmake .." && \
    make -j4 && \
    make install && \
    chown -R $username: /home/$username/yarp-ros2


# Fix default ROS image entrypoint (and bug in randaz81/r1slam:ros2)
RUN sed -i 's|export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/home/user1/navigation/build/bin|export PATH=\$PATH:\$robotology_install_folder/navigation/build/bin|' /home/$username/.bashrc
RUN mkdir -p /opt/ros/none; \
    touch /opt/ros/none/setup.bash; \
    chmod 644 /opt/ros/none/setup.bash
ENV ROS_DISTRO=none
ADD yarp-ros2_entrypoint.sh /
ENTRYPOINT ["/yarp-ros2_entrypoint.sh"]


USER $username
WORKDIR /home/$username
