"""Idempotently add a front camera to dtw_robot.xacro (run inside the container).
usage: python3 add_camera.py [path/to/dtw_robot.xacro]"""
import os, shutil, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    '~/airobot_ws/src/gazebo_urdf_lecture/xacro/dtw_robot.xacro')
s = open(path, encoding='utf-8').read()
if 'camera_link' in s:
    print('camera already present'); sys.exit(0)
BLOCK = '''
  <!-- front camera (added for dtw_vision) -->
  <joint name="camera_joint" type="fixed">
    <parent link="body_link"/><child link="camera_link"/><origin xyz="0.12 0 0.06" rpy="0 0 0"/>
  </joint>
  <link name="camera_link">
    <visual><geometry><box size="0.02 0.04 0.03"/></geometry></visual>
  </link>
  <gazebo reference="camera_link">
    <sensor type="camera" name="front_camera">
      <update_rate>10</update_rate>
      <camera>
        <horizontal_fov>1.2</horizontal_fov>
        <image><width>320</width><height>240</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>20</far></clip>
      </camera>
      <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
        <camera_name>camera</camera_name><frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
'''
i = s.rindex('</robot>')
shutil.copy(path, path + '.bak')
open(path, 'w', encoding='utf-8').write(s[:i] + BLOCK + s[i:])
print('camera added')
