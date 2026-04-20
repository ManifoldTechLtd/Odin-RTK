FROM osrf/ros:humble-desktop
# Args for the non-root user
ARG USERNAME=manifold
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME
# 更换软件源
RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|https://mirrors.aliyun.com/ubuntu/|g' /etc/apt/sources.list && \
    sed -i 's|http://security.ubuntu.com/ubuntu/|https://mirrors.aliyun.com/ubuntu/|g' /etc/apt/sources.list
# 更换ros源
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.aliyun.com/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt upgrade -y
RUN apt-get install -q -y vim tmux gdb cmake zip htop net-tools python3-pip curl git wget libyaml-cpp-dev
RUN apt-get install -q -y  ros-humble-pcl-ros python3-ament-package ros-humble-compressed-image-transport ros-humble-image-transport-plugins
# odin_rtk_fusion 第三方库依赖
RUN apt-get install -q -y  libproj-dev libyaml-cpp-dev libeigen3-dev
RUN apt-get install -q -y  ros-humble-pcl-conversions ros-humble-tf2-ros ros-humble-visualization-msgs
RUN apt-get install -q -y  ros-humble-nmea-msgs
# 安装 Livox-SDK2
WORKDIR /home/${USERNAME}/third_party/
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
WORKDIR /home/${USERNAME}/third_party/Livox-SDK2/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4 && make install
# 安装 livox_ros_driver2
WORKDIR /home/${USERNAME}/ws_livox_driver/src
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git
WORKDIR /home/${USERNAME}/ws_livox_driver/src/livox_ros_driver2
RUN ./build.sh humble
# gtsam4.1.1
WORKDIR /home/${USERNAME}/third_party
RUN git clone https://github.com/borglab/gtsam.git --branch 4.1.1 --single-branch --depth 1
WORKDIR /home/${USERNAME}/third_party/gtsam/build
RUN cmake -G "Unix Makefiles" .. -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON
RUN cmake --build . --target install -- -j 2 && ldconfig
# spdlog
WORKDIR /home/${USERNAME}/third_party/
RUN wget https://github.com/gabime/spdlog/archive/refs/tags/v1.14.1.tar.gz
RUN tar -xzvf v1.14.1.tar.gz
WORKDIR /home/${USERNAME}/third_party/spdlog-1.14.1/build
RUN cmake -G "Unix Makefiles" .. -DCMAKE_BUILD_TYPE=Release
RUN cmake --build . --target install -- -j 2 && ldconfig
# 安装 Sophus
# WORKDIR /home/${USERNAME}/third_party/
# RUN git clone https://github.com/strasdat/Sophus.git
# WORKDIR /home/${USERNAME}/third_party/Sophus
# RUN git checkout a621ff
# RUN sed -i 's/unit_complex_.real() = 1\.;/unit_complex_.real(1.);/g' /home/manifold/third_party/Sophus/sophus/so2.cpp && sed -i 's/unit_complex_.imag() = 0\.;/unit_complex_.imag(0.);/g' /home/manifold/third_party/Sophus/sophus/so2.cpp
# WORKDIR /home/${USERNAME}/third_party/Sophus/build
# RUN cmake .. && make -j4 && make install