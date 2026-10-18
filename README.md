# dtw_vision : AI ball tracking + LiDAR obstacle avoidance

```
camera -> ball_tracker (HSV / YOLOv8n) -> /cmd_vel_raw -> obstacle_avoid -> /cmd_vel_in -> heading_hold -> /cmd_vel
```
Existing: dtw_control (heading_hold, obstacle_avoid), gazebo_urdf_lecture. New: dtw_vision, camera in xacro.

## 1. Send to container (Windows PowerShell)
```powershell
$o = "<this folder>\pai_final"
docker cp "$o\src\dtw_vision" cc8975b1b58c:/home/ubuntu/airobot_ws/src/dtw_vision
docker exec -u root cc8975b1b58c chown -R ubuntu:ubuntu /home/ubuntu/airobot_ws/src/dtw_vision
```
## 2. Container
```bash
pip3 install ultralytics --extra-index-url https://download.pytorch.org/whl/cpu   # YOLO (later)
python3 ~/airobot_ws/src/dtw_vision/scripts/add_camera.py
cd ~/airobot_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select dtw_vision --symlink-install && source install/setup.bash
```
## 3. Run (each in its own terminal; restart Gazebo launch once after add_camera)
```bash
ros2 launch gazebo_urdf_lecture dtw_gazebo.launch.py
ros2 run dtw_control heading_hold --ros-args -p use_sim_time:=true
ros2 run dtw_control obstacle_avoid --ros-args -p use_sim_time:=true
ros2 topic hz /camera/image_raw                      # ~10 Hz expected
ros2 run gazebo_ros spawn_entity.py -entity obstacle -file /tmp/obstacle.sdf -x 2.5 -y 0.0 -z 0.5
ros2 run gazebo_ros spawn_entity.py -entity red_ball -file ~/airobot_ws/src/dtw_vision/models/ball.sdf -x 5 -y 0 -z 0.1
ros2 run dtw_vision ball_mover --ros-args -p use_sim_time:=true
ros2 run dtw_vision ball_tracker --ros-args -p use_sim_time:=true -p detector:=hsv
ros2 run rqt_image_view rqt_image_view /debug_image
```
## 4. AI step
```bash
ros2 run dtw_vision ball_tracker --ros-args -p use_sim_time:=true -p detector:=hsv -p collect_dir:=$HOME/ball_dataset   # ~3 min
python3 ~/airobot_ws/src/dtw_vision/scripts/train.py $HOME/ball_dataset/data.yaml
ros2 run dtw_vision ball_tracker --ros-args -p use_sim_time:=true -p detector:=yolo -p model_path:=runs/ball/weights/best.pt
```
Notes: target_dist (1.2 m) must stay above obstacle_avoid's turn trigger (0.8 m), else it turns away from the ball.
metrics.csv (detection rate, latency, distance error) is written for the report.

## Repo layout
```
src/dtw_vision/            new: AI ball tracker (this work)
src/dtw_control/           heading_hold, obstacle_avoid (copied from container)
src/gazebo_urdf_lecture/   robot model + launch (copied from container)
docs/                      report PDF source, figures
```
Baseline: ROS講座39 (Qiita, @srs) ported to ROS 2 Humble. See report for what was added.
