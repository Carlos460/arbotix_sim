# syntax=docker/dockerfile:1
# docerfile to run on macos
FROM ros:jazzy-ros-core

ENV DEBIAN_FRONTEND=noninteractive

# Add OSRF Gazebo repository
RUN apt-get update && apt-get install -y curl lsb-release gnupg wget python3-rosdep\
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
       -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic, ROS 2 integration, build tools, and the virtual display stack
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    ros-jazzy-ros-gz \
    libgl1 \
    libgl1-mesa-dri \
    mesa-utils \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    openbox \
    python3-colcon-common-extensions \
    cmake \
    make \
    g++ \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    ros-jazzy-ros2bag \
    ros-jazzy-rosbag2-storage-default-plugins \
    && rm -rf /var/lib/apt/lists/*

# Render in software on a virtual display inside the container
ENV DISPLAY=:1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root

# Starts Xvfb + window manager + VNC + noVNC, then runs the container command
COPY --chmod=755 <<'EOF' /usr/local/bin/start-gui.sh
#!/bin/bash
mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
Xvfb :1 -screen 0 1920x1080x24 > /tmp/xvfb.log 2>&1 &
until [ -S /tmp/.X11-unix/X1 ]; do sleep 0.1; done
openbox > /tmp/openbox.log 2>&1 &
x11vnc -display :1 -forever -shared -nopw -quiet -bg -o /tmp/x11vnc.log
websockify --web /usr/share/novnc 6080 localhost:5900 > /tmp/novnc.log 2>&1 &
source /opt/ros/jazzy/setup.bash
exec "$@"
EOF

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws

EXPOSE 6080

ENTRYPOINT ["/usr/local/bin/start-gui.sh"]
CMD ["bash"]
