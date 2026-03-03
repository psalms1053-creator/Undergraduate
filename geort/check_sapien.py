import sapien.core as sapien
import numpy as np
from transforms3d.euler import quat2euler  # 모듈 경로 수정
from sapien.utils import Viewer

# 1. 5, 10, 15, 20번 노드 필터링 및 데이터 로드
manus_raw_data = [
    {"node_id": 0, "parent_node_id": 0, "joint_type": "Invalid", "chain_type": "Hand", 
     "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, 
              "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}},
    
    # Thumb (엄지)
    {"node_id": 1, "parent_node_id": 0, "joint_type": "MCP", "chain_type": "Thumb", 
     "pose": {"position": {"x": -0.02895, "y": -0.01756, "z": 0.03502}, 
              "orientation": {"x": 0.04279, "y": -0.37023, "z": 0.59469, "w": 0.71235}}},
    {"node_id": 2, "parent_node_id": 1, "joint_type": "PIP", "chain_type": "Thumb", 
     "pose": {"position": {"x": -0.05023, "y": -0.03995, "z": 0.06727}, 
              "orientation": {"x": 0.37420, "y": -0.21388, "z": 0.64043, "w": 0.63567}}},
    {"node_id": 3, "parent_node_id": 2, "joint_type": "DIP", "chain_type": "Thumb", 
     "pose": {"position": {"x": -0.04362, "y": -0.06387, "z": 0.08732}, 
              "orientation": {"x": 0.72028, "y": 0.12526, "z": 0.64645, "w": 0.21820}}},
    {"node_id": 4, "parent_node_id": 3, "joint_type": "TIP", "chain_type": "Thumb", 
     "pose": {"position": {"x": -0.01715, "y": -0.06796, "z": 0.08547}, 
              "orientation": {"x": 0.72028, "y": 0.12526, "z": 0.64645, "w": 0.21820}}},

    # Index (검지)
    {"node_id": 5, "parent_node_id": 0, "joint_type": "MCP", "chain_type": "Index", 
     "pose": {"position": {"x": -0.02243, "y": -0.00076, "z": 0.04783}, 
              "orientation": {"x": -0.04394, "y": -0.01002, "z": -0.00044, "w": 0.99898}}},
    {"node_id": 6, "parent_node_id": 5, "joint_type": "PIP", "chain_type": "Index", 
     "pose": {"position": {"x": -0.02360, "y": 0.00437, "z": 0.10609}, 
              "orientation": {"x": 0.61149, "y": -0.00448, "z": -0.00400, "w": 0.79123}}},
    {"node_id": 7, "parent_node_id": 6, "joint_type": "IP", "chain_type": "Index", 
     "pose": {"position": {"x": -0.02422, "y": -0.04592, "z": 0.11919}, 
              "orientation": {"x": 0.98508, "y": -0.00495, "z": 0.00035, "w": 0.17204}}},
    {"node_id": 8, "parent_node_id": 7, "joint_type": "DIP", "chain_type": "Index", 
     "pose": {"position": {"x": -0.02425, "y": -0.05579, "z": 0.09180}, 
              "orientation": {"x": 0.90024, "y": -0.00319, "z": 0.00478, "w": -0.43536}}},
    {"node_id": 9, "parent_node_id": 8, "joint_type": "TIP", "chain_type": "Index", 
     "pose": {"position": {"x": -0.02396, "y": -0.03576, "z": 0.07593}, 
              "orientation": {"x": 0.90024, "y": -0.00319, "z": 0.00478, "w": -0.43536}}},

    # Middle (중지)
    {"node_id": 10, "parent_node_id": 0, "joint_type": "MCP", "chain_type": "Middle", 
     "pose": {"position": {"x": -0.00309, "y": 0.00504, "z": 0.04657}, 
              "orientation": {"x": 0.00981, "y": 0.00926, "z": -0.00009, "w": 0.99991}}},
    {"node_id": 11, "parent_node_id": 10, "joint_type": "PIP", "chain_type": "Middle", 
     "pose": {"position": {"x": -0.00199, "y": 0.00388, "z": 0.10575}, 
              "orientation": {"x": 0.64477, "y": -0.01741, "z": 0.01959, "w": 0.76393}}},
    {"node_id": 12, "parent_node_id": 11, "joint_type": "IP", "chain_type": "Middle", 
     "pose": {"position": {"x": -0.00206, "y": -0.04751, "z": 0.11450}, 
              "orientation": {"x": 0.99747, "y": -0.00225, "z": 0.02436, "w": 0.06676}}},
    {"node_id": 13, "parent_node_id": 12, "joint_type": "DIP", "chain_type": "Middle", 
     "pose": {"position": {"x": -0.00061, "y": -0.05152, "z": 0.08473}, 
              "orientation": {"x": 0.83611, "y": 0.01116, "z": 0.01346, "w": -0.54828}}},
    {"node_id": 14, "parent_node_id": 13, "joint_type": "TIP", "chain_type": "Middle", 
     "pose": {"position": {"x": -0.00036, "y": -0.02909, "z": 0.07499}, 
              "orientation": {"x": 0.83611, "y": 0.01116, "z": 0.01346, "w": -0.54828}}},

    # Ring (약지)
    {"node_id": 15, "parent_node_id": 0, "joint_type": "MCP", "chain_type": "Ring", 
     "pose": {"position": {"x": 0.01009, "y": 0.00486, "z": 0.04674}, 
              "orientation": {"x": 0.01642, "y": 0.07203, "z": -0.00119, "w": 0.99727}}},
    {"node_id": 16, "parent_node_id": 15, "joint_type": "PIP", "chain_type": "Ring", 
     "pose": {"position": {"x": 0.01767, "y": 0.00312, "z": 0.09891}, 
              "orientation": {"x": 0.63422, "y": -0.01516, "z": 0.01166, "w": 0.77291}}},
    {"node_id": 17, "parent_node_id": 16, "joint_type": "IP", "chain_type": "Ring", 
     "pose": {"position": {"x": 0.01725, "y": -0.04420, "z": 0.10832}, 
              "orientation": {"x": 0.99409, "y": -0.01293, "z": 0.01490, "w": 0.10676}}},
    {"node_id": 18, "parent_node_id": 17, "joint_type": "DIP", "chain_type": "Ring", 
     "pose": {"position": {"x": 0.01806, "y": -0.05061, "z": 0.07888}, 
              "orientation": {"x": 0.86790, "y": -0.00699, "z": 0.00348, "w": -0.49669}}},
    {"node_id": 19, "parent_node_id": 18, "joint_type": "TIP", "chain_type": "Ring", 
     "pose": {"position": {"x": 0.01838, "y": -0.02968, "z": 0.06658}, 
              "orientation": {"x": 0.86790, "y": -0.00699, "z": 0.00348, "w": -0.49669}}},

    # Pinky (소지)
    {"node_id": 20, "parent_node_id": 0, "joint_type": "MCP", "chain_type": "Pinky", 
     "pose": {"position": {"x": 0.02289, "y": 0.00425, "z": 0.04618}, 
              "orientation": {"x": 0.03315, "y": 0.16414, "z": -0.00552, "w": 0.98586}}},
    {"node_id": 21, "parent_node_id": 20, "joint_type": "PIP", "chain_type": "Pinky", 
     "pose": {"position": {"x": 0.04006, "y": 0.00068, "z": 0.09632}, 
              "orientation": {"x": 0.71824, "y": -0.02224, "z": -0.00909, "w": 0.69538}}},
    {"node_id": 22, "parent_node_id": 21, "joint_type": "IP", "chain_type": "Pinky", 
     "pose": {"position": {"x": 0.03851, "y": -0.03466, "z": 0.09516}, 
              "orientation": {"x": 0.98953, "y": -0.05753, "z": -0.00985, "w": 0.13197}}},
    {"node_id": 23, "parent_node_id": 22, "joint_type": "DIP", "chain_type": "Pinky", 
     "pose": {"position": {"x": 0.03786, "y": -0.03947, "z": 0.07727}, 
              "orientation": {"x": 0.92870, "y": -0.07746, "z": -0.03183, "w": -0.36124}}},
    {"node_id": 24, "parent_node_id": 23, "joint_type": "TIP", "chain_type": "Pinky", 
     "pose": {"position": {"x": 0.03780, "y": -0.02592, "z": 0.06249}, 
              "orientation": {"x": 0.92870, "y": -0.07746, "z": -0.03183, "w": -0.36124}}}
]
filtered_nodes = [n for n in manus_raw_data if n['node_id'] not in [5, 10, 15, 20]]

def apply_manus_to_robot(robot, nodes):
    # 로봇의 모든 active joint 이름을 가져와 매핑 준비
    joints = robot.get_active_joints()
    joint_names = [j.get_name() for j in joints]
    qpos = robot.get_qpos()

    # Manus Node ID와 URDF 조인트 이름 매핑 (test_fixed.urdf 분석 결과)
    # 각 관절은 주로 z축 회전을 담당함
    mapping = {
        # Index (검지)
        6: "joint1_index_z", 7: "joint2_index_z", 8: "joint3_index_z",
        # Middle (중지)
        11: "joint1_middle_z", 12: "joint2_middle_z", 13: "joint3_middle_z",
        # Ring (약지)
        16: "joint1_ring_z", 17: "joint2_ring_z", 18: "joint3_ring_z",
        # Pinky (소지)
        21: "joint1_baby_z", 22: "joint2_baby_z", 23: "joint3_baby_z",
        # Thumb (엄지 - 구조에 따라 추가 조정 필요)
        2: "joint1_thumb_z", 3: "joint2_thumb_z"
    }

    for node in nodes:
        nid = node['node_id']
        if nid in mapping:
            target_joint_name = mapping[nid]
            if target_joint_name in joint_names:
                idx = joint_names.index(target_joint_name)
                
                # 쿼터니언 추출 [x, y, z, w] -> transforms3d는 [w, x, y, z] 순서 사용
                q = node['pose']['orientation']
                quat_wxyz = [q['w'], q['x'], q['y'], q['z']]
                
                # Euler 변환 (상대적 회전량 추출)
                # 로봇의 관절 축(z)에 해당하는 각도 추출
                euler = quat2euler(quat_wxyz)
                joint_angle = euler[2] # Z축 회전값 사용
                
                # URDF의 limit 범위 내로 클리핑 (부정확한 설계 확인용)
                qpos[idx] = joint_angle

    # 로봇에 최종 포즈 적용
    robot.set_qpos(qpos)

def main():
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)
    scene = engine.create_scene()
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    loader = scene.create_urdf_loader()
    # 경로를 실제 환경에 맞춰 확인하세요
    robot = loader.load("/home/jy/ros2_ws/src/GeoRT/assets/dg5f_right/dg5f_right.urdf")
    robot.set_root_pose(sapien.Pose([0, 0, 0]))

    # 1. 로봇에게 Manus 포즈 입히기
    apply_manus_to_robot(robot, filtered_nodes)

    # 2. Manus 원본 위치(빨간 점) 시각화 (비교용)
    SCALE_FACTOR = 1.5
    for node in filtered_nodes:
        pos = node['pose']['position']
        # 좌표계 변환: Manus -> SAPIEN/URDF
        target_pos = np.array([-pos['y'], -pos['x'], pos['z']]) * SCALE_FACTOR
        
        builder = scene.create_actor_builder()
        builder.add_sphere_visual(radius=0.005, color=[1, 0, 0, 1])
        visual_node = builder.build_static()
        visual_node.set_pose(sapien.Pose(target_pos))

    viewer = Viewer(renderer)
    viewer.set_scene(scene)
    
    print("로봇이 Manus 포즈를 취했습니다. 빨간 점(원본)과 로봇 마디의 일치 여부를 확인하세요.")
    
    while not viewer.closed:
        scene.step()
        scene.update_render()
        viewer.render()

if __name__ == "__main__":
    main()