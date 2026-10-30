# ros2_bridge

Shows the hexapod in Gazebo and moves it with joint angles sent from the
ArbotiX simulator over UDP.

```
arbotix_gui UDP :15000 --> udp_joint_bridge --> Gazebo
```

The robot model is derived from
[HumaRobotics/phantomx_description](https://github.com/HumaRobotics/phantomx_description/tree/master).
This includes the hexapod_urdf.xacro and the /meshes directory.

## Run on Linux

Install ROS 2 Jazzy [Doc link](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

Requires ROS 2 Jazzy and these packages (Gazebo Harmonic comes with `ros-jazzy-ros-gz`):

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher
```

From this directory:

Make sure to source ROS 2 or add to .bashrc
```
source /opt/ros/jazzy/setup.bash
```

```bash
colcon build && source install/setup.bash
ros2 launch cpp_pubsub hexapod_gazebo.launch.py
```

This starts Gazebo, adds the robot, and starts `udp_joint_bridge`. Then send
joint data from `arbotix_gui` or with the [test script](#test-without-the-simulator).

Re-run `colcon build` after changing `hexapod_urdf.xacro` or `meshes/`, since
the launch file uses the installed copies.

## Run in Docker (macOS)

Gazebo runs on a virtual display in the container and is viewed in a browser.

```bash
docker build -t ros2_ws .
docker rm -f ros2_ws_dev   # only if an old container exists
docker run -d --name ros2_ws_dev \
  -v "$(pwd):/ros2_ws" \
  -p 127.0.0.1:6080:6080 \
  -p 127.0.0.1:15000:15000/udp \
  ros2_ws sleep infinity
```

Port 6080 is the browser view and port 15000 receives joint data.
Open a shell in the container, then build and launch as in [Run on Linux](#run-on-linux):

```bash
docker exec -it ros2_ws_dev bash
```

To see Gazebo, open http://localhost:6080/vnc.html and click **Connect**.

Stop and remove the container with `docker rm -f ros2_ws_dev`.

## Joint data format

Each UDP packet holds one or more 8-byte records:

```cpp
struct joint_packet {
  int32_t joint_id;      // servo ID 1..18
  float joint_position;  // radians, 0 = servo center (tick 512)
};
```

Servo IDs follow the standard PhantomX layout:

| Leg          | Coxa | Femur | Tibia |
| ------------ | ---- | ----- | ----- |
| right front  | 1    | 3     | 5     |
| left front   | 2    | 4     | 6     |
| right rear   | 7    | 9     | 11    |
| left rear    | 8    | 10    | 12    |
| right middle | 13   | 15    | 17    |
| left middle  | 14   | 16    | 18    |

`udp_joint_bridge` settings, passed with `--ros-args -p name:=value`:

| Setting           | Default    | What it does                                        |
| ----------------- | ---------- | --------------------------------------------------- |
| `port`            | `15000`    | UDP port to listen on                               |
| `bind_address`    | `0.0.0.0`  | Address to listen on                                |
| `publish_rate_hz` | `50.0`     | How often joint angles are sent to Gazebo           |
| `joint_names`     | table above | Joint name for each servo ID, in ID order          |
| `joint_signs`     | all `1.0`  | Set an entry to `-1.0` to reverse that joint        |

## Test without the simulator

`src/cpp_pubsub/scripts/send_test_joints.py` swings every joint back and forth
so you can check the setup without running `arbotix_gui`:

```bash
python3 src/cpp_pubsub/scripts/send_test_joints.py [host] [port]   # default 127.0.0.1 15000
```

From macOS to Docker, run it on the Mac with the defaults.

## Files

| Path                                       | What it is                                       |
| ------------------------------------------ | ------------------------------------------------ |
| `hexapod_urdf.xacro`                       | Robot model, including Gazebo joint controllers  |
| `meshes/`                                  | 3D models used by the robot model                |
| `src/cpp_pubsub/src/udp_joint_bridge.cpp`  | Receives UDP joint data and sends it to Gazebo   |
| `src/cpp_pubsub/launch/hexapod_gazebo.launch.py` | Starts everything with one command         |
| `src/cpp_pubsub/config/gz_bridge.yaml`     | Which ROS topics are passed through to Gazebo    |
| `src/cpp_pubsub/scripts/send_test_joints.py` | Sends test joint data                          |
