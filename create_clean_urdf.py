import os

# 파일 경로 (진단 스크립트에서 확인된 경로)
path = "/home/jy/ros2_ws/src/GeoRT/assets/ASSY_Hand_R/test_fixed.urdf"

# 올바른 URDF 내용 (Mesh 경로 상대경로 적용됨)
urdf_content = r"""<?xml version="1.0" encoding="utf-8"?>
<robot name="aidin_hand_fixed">

  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.001"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>

  <link name="body">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="meshes/base_link.STL" />
      </geometry>
      <material name="">
        <color rgba="0.2 0.2 0.2 1" />
      </material>
    </visual>
  </link>

  <joint name="base_to_body" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 -1.5708"/>
    <parent link="base_link"/>
    <child link="body"/>
  </joint>

  <link name="link1_thumb">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="meshes/link1_thumb.STL" />
      </geometry>
      <material name="">
        <color rgba="0.2 0.2 0.2 1" />
      </material>
    </visual>
  </link>
  <joint name="joint1_thumb_z" type="revolute">
    <origin xyz="0.033 0.0195 0.0276" rpy="0 1.5708 0" />
    <parent link="body" />
    <child link="link1_thumb" />
    <axis xyz="0 0 1" />
    <limit lower="-0.5236" upper="0.5236" effort="10" velocity="1.57" />
  </joint>

  <link name="link2_thumb">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="meshes/link2_thumb.STL" />
      </geometry>
      <material name="">
        <color rgba="0.2 0.2 0.2 1" />
      </material>
    </visual>
  </link>
  <joint name="joint2_thumb_z" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="link1_thumb" />
    <child link="link2_thumb" />
    <axis xyz="0 0 1" />
    <limit lower="-0.785" upper="0.785" effort="10" velocity="1.57" />
  </joint>

  <link name="link3_thumb">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="meshes/link3_thumb.STL" />
      </geometry>
      <material name="">
        <color rgba="0.2 0.2 0.2 1" />
      </material>
    </visual>
  </link>
  <joint name="joint3_thumb_z" type="revolute">
    <origin xyz="0.046 0 0" rpy="0 0 0" />
    <parent link="link2_thumb" />
    <child link="link3_thumb" />
    <axis xyz="0 0 1" />
    <limit lower="-0.174" upper="1.57" effort="10" velocity="1.57" />
  </joint>

  <link name="link4_thumb">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="meshes/link4_thumb.STL" />
      </geometry>
      <material name="">
        <color rgba="0.7 0.7 0.7 1" />
      </material>
    </visual>
  </link>
  <joint name="joint4_thumb_z" type="revolute">
    <origin xyz="0.046 0 0" rpy="0 0 0" />
    <parent link="link3_thumb" />
    <child link="link4_thumb" />
    <axis xyz="0 0 -1" />
    <limit lower="0" upper="1.134" effort="10" velocity="1.57" />
  </joint>

  <link name="link1_index">
    <visual>
      <geometry><mesh filename="meshes/link1_index.STL" /></geometry>
    </visual>
  </link>
  <joint name="joint1_index_z" type="revolute">
    <origin xyz="0.033 0.0515 0.0036" rpy="1.5708 0 3.14159" />
    <parent link="body" />
    <child link="link1_index" />
    <axis xyz="0 0 1" />
    <limit lower="-0.5236" upper="0.5236" effort="10" velocity="1.57" />
  </joint>

  <link name="link2_index">
    <visual>
      <geometry><mesh filename="meshes/link2_index.STL" /></geometry>
    </visual>
  </link>
  <joint name="joint2_index_z" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="link1_index" />
    <child link="link2_index" />
    <axis xyz="0 0 1" />
    <limit lower="0" upper="1.57" effort="10" velocity="1.57" />
  </joint>

  <link name="link3_index">
    <visual>
      <geometry><mesh filename="meshes/link3_index.STL" /></geometry>
    </visual>
  </link>
  <joint name="joint3_index_z" type="revolute">
    <origin xyz="0.045 0 0" rpy="0 0 0" />
    <parent link="link2_index" />
    <child link="link3_index" />
    <axis xyz="0 0 1" />
    <limit lower="0" upper="1.57" effort="10" velocity="1.57" />
  </joint>

  <link name="link4_index">
    <visual>
      <geometry><mesh filename="meshes/link4_index.STL" /></geometry>
    </visual>
  </link>
  <joint name="joint4_index_z" type="revolute">
    <origin xyz="0.03 0 0" rpy="0 0 0" />
    <parent link="link3_index" />
    <child link="link4_index" />
    <axis xyz="0 0 -1" />
    <limit lower="0" upper="1.134" effort="10" velocity="1.57" />
  </joint>

  <link name="link1_middle">
    <visual>
      <geometry><mesh filename="meshes/link1_middle.STL" /></geometry>
    </visual>
  </link>
  <joint name="joint1_middle_z" type="revolute">
    <origin xyz="0.033 0.0195 0.0036" rpy="1.5708 0 3.14159" />
    <parent link="body" />
    <child link="link1_middle" />
    <axis xyz="0 0 1" />
    <limit lower="-0.5236" upper="0.5236" effort="10" velocity="1.57" />
  </joint>

  <link name="link2_middle">
    <visual>
      <geometry><mesh filename="meshes/link2_middle.STL" /></geometry>
    </visual>
  </link>
  <joint name="joint2_middle_z" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="link1_middle" />
    <child link="link2_middle" />
    <axis xyz="0 0 1" />
    <limit lower="0" upper="1.57" effort="10" velocity="1.57" />
  </joint>

  <link name="link3_middle">
    <visual>
      <geometry><mesh filename="meshes/link3_middle.STL" /></geometry>
    </visual>
  </link>
  <joint name="joint3_middle_z" type="revolute">
    <origin xyz="0.045 0 0" rpy="0 0 0" />
    <parent link="link2_middle" />
    <child link="link3_middle" />
    <axis xyz="0 0 1" />
    <limit lower="0" upper="1.57" effort="10" velocity="1.57" />
  </joint>

  <link name="link4_middle">
    <visual>
      <geometry><mesh filename="meshes/link4_middle.STL" /></geometry>
    </visual>
  </link>
  <joint name="joint4_middle_z" type="revolute">
    <origin xyz="0.03 0 0" rpy="0 0 0" />
    <parent link="link3_middle" />
    <child link="link4_middle" />
    <axis xyz="0 0 -1" />
    <limit lower="0" upper="1.134" effort="10" velocity="1.57" />
  </joint>

  <link name="link1_ring">
    <visual>
      <geometry><mesh filename="meshes/link1_ring.STL" /></geometry>
    </visual>
  </link>
  <joint name="joint1_ring_z" type="revolute">
    <origin xyz="0.033 -0.0125 0.0036" rpy="1.5708 0 3.14159" />
    <parent link="body" />
    <child link="link1_ring" />
    <axis xyz="0 0 1" />
    <limit lower="-0.5236" upper="0.5236" effort="10" velocity="1.57" />
  </joint>

  <link name="link2_ring">
    <visual>
      <geometry><mesh filename="meshes/link2_ring.STL" /></geometry>
    </visual>
  </link>
  <joint name="joint2_ring_z" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="link1_ring" />
    <child link="link2_ring" />
    <axis xyz="0 0 1" />
    <limit lower="0" upper="1.57" effort="10" velocity="1.57" />
  </joint>

  <link name="link3_ring">
    <visual>
      <geometry><mesh filename="meshes/link3_ring.STL" /></geometry>
    </visual>
  </link>
  <joint name="joint3_ring_z" type="revolute">
    <origin xyz="0.045 0 0" rpy="0 0 0" />
    <parent link="link2_ring" />
    <child link="link3_ring" />
    <axis xyz="0 0 1" />
    <limit lower="0" upper="1.57" effort="10" velocity="1.57" />
  </joint>

  <link name="link4_ring">
    <visual>
      <geometry><mesh filename="meshes/link4_ring.STL" /></geometry>
    </visual>
  </link>
  <joint name="joint4_ring_z" type="revolute">
    <origin xyz="0.03 0 0" rpy="0 0 0" />
    <parent link="link3_ring" />
    <child link="link4_ring" />
    <axis xyz="0 0 -1" />
    <limit lower="0" upper="1.134" effort="10" velocity="1.57" />
  </joint>

  <link name="link1_baby">
    <visual>
      <geometry><mesh filename="meshes/link1_baby.STL" /></geometry>
    </visual>
  </link>
  <joint name="joint1_baby_z" type="revolute">
    <origin xyz="0.033 -0.0445 0.0036" rpy="1.5708 0 3.14159" />
    <parent link="body" />
    <child link="link1_baby" />
    <axis xyz="0 0 1" />
    <limit lower="-0.5236" upper="0.5236" effort="10" velocity="1.57" />
  </joint>

  <link name="link2_baby">
    <visual>
      <geometry><mesh filename="meshes/link2_baby.STL" /></geometry>
    </visual>
  </link>
  <joint name="joint2_baby_z" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="link1_baby" />
    <child link="link2_baby" />
    <axis xyz="0 0 1" />
    <limit lower="0" upper="1.57" effort="10" velocity="1.57" />
  </joint>

  <link name="link3_baby">
    <visual>
      <geometry><mesh filename="meshes/link3_baby.STL" /></geometry>
    </visual>
  </link>
  <joint name="joint3_baby_z" type="revolute">
    <origin xyz="0.045 0 0" rpy="0 0 0" />
    <parent link="link2_baby" />
    <child link="link3_baby" />
    <axis xyz="0 0 1" />
    <limit lower="0" upper="1.57" effort="10" velocity="1.57" />
  </joint>

  <link name="link4_baby">
    <visual>
      <geometry><mesh filename="meshes/link4_baby.STL" /></geometry>
    </visual>
  </link>
  <joint name="joint4_baby_z" type="revolute">
    <origin xyz="0.03 0 0" rpy="0 0 0" />
    <parent link="link3_baby" />
    <child link="link4_baby" />
    <axis xyz="0 0 -1" />
    <limit lower="0" upper="1.134" effort="10" velocity="1.57" />
  </joint>

</robot>
"""

# 파일 쓰기 (기존 파일 덮어씀)
with open(path, "w", encoding="utf-8") as f:
    f.write(urdf_content)

print(f"✅ 파일을 성공적으로 재생성했습니다: {path}")
print("   -> 이제 시뮬레이션을 다시 실행해 보세요.")
