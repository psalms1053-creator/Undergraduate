import sapien.core as sapien
from sapien.utils.viewer import Viewer
import numpy as np
import math

def main():
    # 1. SAPIEN 셋업
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)
    scene = engine.create_scene()
    scene.set_timestep(1 / 100.0)
    
    scene.add_directional_light([0, 1, -1], [1, 1, 1])
    # 수정 완료: add_ambient_light -> set_ambient_light
    scene.set_ambient_light([0.5, 0.5, 0.5])

    # 2. 뷰어(GUI) 설정
    viewer = Viewer(renderer)
    viewer.set_scene(scene)
    viewer.set_camera_xyz(x=0.3, y=0.0, z=0.2)
    viewer.set_camera_rpy(r=0, p=-0.5, y=3.14)

    # 3. URDF 로드 (중복 선언 오류 해결)
    urdf_path = "/home/jy/ros2_ws/src/GeoRT/assets/ASSY_Hand_R/test_fixed.urdf"
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True # 손이 바닥으로 떨어지지 않게 고정
    robot = loader.load(urdf_path)

    # 4. 제어할 조인트 찾기
    joint_names = [j.name for j in robot.get_active_joints()]
    print("로드된 조인트 목록:", joint_names)
    
    target_joint_name = "joint2_index_z"
    if target_joint_name not in joint_names:
        print(f"에러: {target_joint_name}을(를) 찾을 수 없습니다!")
        return

    target_joint_idx = joint_names.index(target_joint_name)

    # manus_evaluation.py 의 30번째 줄 부근 주석 해제
    for link in robot.get_links():
        for shape in link.get_collision_shapes():
            shape.set_collision_groups(0, 0, 0, 0)

    # 5. 시뮬레이션 루프 (손가락 까딱거리기)
    step = 0
    print(f"\n[{target_joint_name}] 구동 테스트를 시작합니다. 뷰어 창을 확인하세요!")
    print("종료하려면 뷰어 창을 닫으세요.")
    
    while not viewer.closed:
        # 사인(sin) 함수를 이용해 0 ~ 1.5 라디안 사이를 왕복하도록 각도 생성
        angle = (math.sin(step * 0.05) + 1.0) * 0.75 
        
        # 현재 로봇의 모든 관절 각도를 가져와서 타겟 관절만 각도 수정
        qpos = robot.get_qpos()
        qpos[target_joint_idx] = angle
        qpos[joint_names.index("joint1_index_z")] = 0.0
        robot.set_qpos(qpos)
        
        # 물리 엔진 1스텝 진행 및 화면 업데이트
        scene.step()
        scene.update_render()
        viewer.render()
        step += 1

if __name__ == '__main__':
    main()