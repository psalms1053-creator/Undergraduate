import sapien.core as sapien
from sapien.utils.viewer import Viewer
import numpy as np
import xml.etree.ElementTree as ET
import os

def get_urdf_mimic_and_limits(urdf_path, active_joint_names):
    """URDF를 파싱하여 Mimic 관계와 관절 Limit(한계값)을 추출하는 함수입니다."""
    mimic_rules = []
    limits = {}
    
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    for joint in root.findall('joint'):
        joint_name = joint.get('name')
        if joint_name not in active_joint_names:
            continue

        limit_tag = joint.find('limit')
        if limit_tag is not None:
            lower = float(limit_tag.get('lower', '-3.14'))
            upper = float(limit_tag.get('upper', '3.14'))
            limits[joint_name] = (lower, upper)

        mimic_tag = joint.find('mimic')
        if mimic_tag is not None:
            parent_name = mimic_tag.get('joint')
            multiplier = float(mimic_tag.get('multiplier', '1.0'))
            offset = float(mimic_tag.get('offset', '0.0'))
            
            if parent_name in active_joint_names:
                mimic_rules.append({
                    'child': joint_name,
                    'parent': parent_name,
                    'multiplier': multiplier,
                    'offset': offset
                })
                
    return mimic_rules, limits

def main():
    urdf_path = "/home/jy/ros2_ws/src/GeoRT/assets/ASSY_Hand_R/test_fixed.urdf"
    if not os.path.exists(urdf_path):
        print(f"❌ URDF 파일을 찾을 수 없습니다: {urdf_path}")
        return

    # =====================================================================
    # 🎯 [사용자 설정 영역] 0번부터 19번 관절까지의 목표 각도를 배열로 설정하세요!
    # =====================================================================
    # 기본적으로 20개의 0.0으로 채워진 배열을 만듭니다.
    TARGET_ANGLES = [0.52, 0.0, 1.5, 0.0,
                     0.0, 2.3, 1.9, 0.0,
                     0.0, 1.9, 1.5, 0.0,
                     0.0, 1.9, 1.5, 0.0,
                     0.0, 1.9, 1.9, 0.0]
    
    # 원하는 관절의 인덱스에 값을 덮어씁니다. (예: 6번 관절을 0.2로 설정)
    #TARGET_ANGLES[6] = 0.2
    
    # 💡 직접 리스트를 쭉 적으셔도 됩니다!
    # TARGET_ANGLES = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 
    #                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # =====================================================================

    # 1. SAPIEN 셋업
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)
    scene = engine.create_scene()
    
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    robot = loader.load(urdf_path)

    # 2. 뷰어 및 카메라 셋업
    viewer = Viewer(renderer)
    viewer.set_scene(scene) 
    viewer.window.set_camera_position([0.1550926, -0.1623763, 0.7064089])
    viewer.window.set_camera_rotation([0.8716827, 0.3260138, 0.12817779, 0.3427167])
    viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)

    # 3. 관절 정보 추출
    active_joints = robot.get_active_joints()
    joint_names = [j.get_name() for j in active_joints]
    name_to_idx = {name: i for i, name in enumerate(joint_names)}
    mimic_rules, limits = get_urdf_mimic_and_limits(urdf_path, joint_names)

    print("\n" + "="*50)
    print("🤖 관절 번호 목록 🤖")
    for i, name in enumerate(joint_names):
        print(f"[{i:2d}] {name}")
    print("="*50)

    # 4. 💡 목표 각도 배열 적용 (렌더링 시작 전 1회 세팅)
    qpos = robot.get_qpos()
    
    # 입력한 배열 길이(20)와 실제 로봇의 관절 개수 중 작은 것까지만 반복 적용 (에러 방지)
    num_joints_to_set = min(len(TARGET_ANGLES), len(joint_names))
    
    for i in range(num_joints_to_set):
        target_name = joint_names[i]
        target_angle = TARGET_ANGLES[i]
        
        # 각도 한계(Limit) 클리핑 적용
        if target_name in limits:
            lower, upper = limits[target_name]
            target_angle = np.clip(target_angle, lower, upper)
            
        qpos[i] = target_angle

    # 종속(Mimic) 관절 계산 (부모 관절의 각도를 기준으로 자식 관절 각도 덮어쓰기)
    for rule in mimic_rules:
        if rule['parent'] in name_to_idx and rule['child'] in name_to_idx:
            parent_idx = name_to_idx[rule['parent']]
            child_idx = name_to_idx[rule['child']]
            qpos[child_idx] = qpos[parent_idx] * rule['multiplier'] + rule['offset']

    # 로봇에 최종 계산된 전체 각도 배열 세팅!
    robot.set_qpos(qpos)
    print("✅ TARGET_ANGLES 배열이 로봇에 성공적으로 적용되었습니다.")
    print("🚀 SAPIEN 뷰어가 실행되었습니다. 마우스로 자유롭게 화면을 확인하세요.")

    # 5. 💡 메인 루프: 입력을 받지 않고 오직 렌더링만 처리
    while not viewer.closed:
        scene.update_render()
        viewer.render()

if __name__ == '__main__':
    main()