FROM ubuntu:20.04

LABEL maintainer="xirhxq"

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV DONT_RUN=1

RUN apt-get update && \
    apt-get install -y \
    lsb-release \
    sudo \
    curl \
    gnupg2 \
    tzdata \
    vim \
    wget \
    python3-pip \
    git \
    build-essential && \
    rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install -y ros-noetic-desktop-full && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN apt-get update && \
    apt-get install -y \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    gazebo11 \
    libgazebo11-dev && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y \
    cmake \
    g++ \
    git

#RUN git clone https://github.com/JSBSim-Team/jsbsim.git /usr/src/jsbsim && \
RUN git clone https://gitee.com/scienceasdf/jsbsim.git /usr/src/jsbsim && \
    cd /usr/src/jsbsim && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install

ENV PX4_HOME=/usr/src/PX4-Autopilot

#RUN git clone https://github.com/PX4/PX4-Autopilot.git ${PX4_HOME} && \

#RUN git clone https://gitee.com/px4-mirror/PX4-Autopilot.git ${PX4_HOME} && \
#    cd ${PX4_HOME} && \
#    git checkout 57898036654da32d6a80f228f79f9543ad8cb1f0

#RUN cd ${PX4_HOME} && \
#    git submodule sync --recursive && \
#    git submodule update --init --recursive

#RUN if [ "$(uname -m)" = "aarch64" ]; then \
 #   cd ${PX4_HOME} && \
 #   sed -i '/gcc-multilib/d' ./Tools/setup/ubuntu.sh && \
 #   sed -i '/g++-multilib/d' ./Tools/setup/ubuntu.sh && \
 #   bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools && \
 #   apt-get install -y gcc-8 g++-8 && \
 #   update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 100 --slave /usr/bin/g++ g++ /usr/bin/g++-8 && \
  #  update-alternatives --config gcc; \
#else \
 #   cd ${PX4_HOME} && \
   # bash ./Tools/setup/ubuntu.sh; \
#fi

# 1. 克隆主仓库 (保持不变)
RUN git clone https://gitee.com/px4-mirror/PX4-Autopilot.git ${PX4_HOME} && \
    cd ${PX4_HOME} && \
    git checkout 57898036654da32d6a80f228f79f9543ad8cb1f0

# 2. 同步子模块 (保持不变，这是最吃网络的一步)
RUN cd ${PX4_HOME} && \
    git submodule sync --recursive && \
    git submodule update --init --recursive

# 3. 核心修改：安装 Gazebo 必须的系统依赖
# 这一步提前装好基础库，防止 ubuntu.sh 脚本运行太久且不透明
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    libgazebo11-dev \
    gazebo11 \
    libprotobuf-dev \
    protobuf-compiler \
    && apt-get clean

# 4. 修改 ubuntu.sh 执行逻辑 (去掉了 --no-sim-tools)
RUN cd ${PX4_HOME} && \
    if [ "$(uname -m)" = "aarch64" ]; then \
        sed -i '/gcc-multilib/d' ./Tools/setup/ubuntu.sh && \
        sed -i '/g++-multilib/d' ./Tools/setup/ubuntu.sh && \
        # ARM 架构下：保留仿真工具安装
        bash ./Tools/setup/ubuntu.sh --no-nuttx; \
    else \
        # x86 架构下：保留仿真工具安装
        bash ./Tools/setup/ubuntu.sh --no-nuttx; \
    fi

# 5. aarch64 编译器切换 (保持不变)
RUN if [ "$(uname -m)" = "aarch64" ]; then \
        apt-get update && \
        apt-get install -y gcc-8 g++-8 && \
        update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 100 --slave /usr/bin/g++ g++ /usr/bin/g++-8 && \
        update-alternatives --set gcc /usr/bin/gcc-8; \
    fi
    
# 6. 最终编译
RUN cd ${PX4_HOME} && \
    make px4_sitl_default gazebo

RUN apt-get install -y tmuxinator
RUN apt-get install -y python3-catkin-tools

RUN useradd -m developer && \
echo "developer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN apt-get update && \
    apt-get install -y geographiclib-tools && \
    geographiclib-get-geoids egm96-5 && \
    geographiclib-get-gravity egm96 && \
    geographiclib-get-magnetic emm2015

USER developer
WORKDIR /home/developer

ENV DONT_RUN=0

RUN echo '#!/bin/bash' > /tmp/setup_environment.sh && \
    echo 'source /opt/ros/noetic/setup.bash' >> /tmp/setup_environment.sh && \
    echo 'source /home/developer/.bashrc' >> /tmp/setup_environment.sh && \
    echo 'source /app/guidance/ros_ws/devel/setup.bash' >> /tmp/setup_environment.sh && \
    echo 'source ${PX4_HOME}/Tools/simulation/gazebo-classic/setup_gazebo.bash ${PX4_HOME} ${PX4_HOME}/build/px4_sitl_default' >> /tmp/setup_environment.sh && \
    echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PX4_HOME}' >> /tmp/setup_environment.sh && \
    echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PX4_HOME}/Tools/simulation/gazebo-classic/sitl_gazebo-classic' >> /tmp/setup_environment.sh && \
    echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins' >> /tmp/setup_environment.sh && \
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${PX4_HOME}/build/px4_sitl_default/build_gazebo' >> /tmp/setup_environment.sh && \
    echo 'export GAZEBO_MODEL_PATH=${PX4_HOME}/Tools/simulation/gazebo-classic/models' >> /tmp/setup_environment.sh && \
    echo 'export GAZEBO_RESOURCE_PATH=${PX4_HOME}/Tools/simulation/gazebo-classic' >> /tmp/setup_environment.sh && \
    chmod +x /tmp/setup_environment.sh

RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install jinja2 scipy && \
    python3 -m pip install --upgrade matplotlib && \
    python3 -m pip install rich tqdm

SHELL ["/bin/bash", "-c"]

USER developer

CMD ["/bin/bash", "-c", "source /tmp/setup_environment.sh && /bin/bash"]
