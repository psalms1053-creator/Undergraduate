import torch
import numpy as np
import sapien.core as sapien
from geort.model import FKModel
import os

# ================= 사용자 설정 =================
URDF_PATH = "/home/jy/ros2_ws/src/GeoRT/assets/ASSY_Hand_R/test_fixed_delmic.urdf"
CKPT_PATH = "/home/jy/ros2_ws/src/GeoRT/checkpoint/fk_model_aidin_right_mimic.pth"
NUM_SAMPLES = 1000  # 1000번 테스트
# ==============================================

def verify_precision():
    print(f"🔬 모델 정밀도 최종 점검 (N={NUM_SAMPLES})...")

    # 1. SAPIEN 엔진 및 로봇 로드
    engine = sapien.Engine()
    scene = engine.create_scene()
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    try:
        robot = loader.load(URDF_PATH)
    except Exception as e:
        print(f"❌ URDF 로드 실패: {e}")
        return

    # 2. 관절 및 링크 매핑 정의 (매우 중요)
    # 모델의 각 네트워크가 담당하는 관절 인덱스
    keypoint_joints = [
        [0, 1, 2, 3],     # Finger 1
        [4, 5, 6, 7],     # Finger 2
        [8, 9, 10, 11],   # Finger 3
        [12, 13, 14, 15], # Finger 4
        [16, 17, 18, 19]  # Finger 5 (Thumb)
    ]
    
    # 각 체인의 "끝점(Target Link)" 정의
    active_joints = robot.get_active_joints()
    target_links = []
    finger_names = []
    
    print("\n📋 [검증 대상 매핑 확인]")
    for i, group in enumerate(keypoint_joints):
        last_joint = active_joints[group[-1]]
        tip_link = last_joint.get_child_link()
        target_links.append(tip_link)
        finger_name = tip_link.name
        finger_names.append(finger_name)
        print(f"   👉 Group {i+1}: Joints {group} -> Target Link: '{finger_name}'")

    # 3. 모델 로드
    fk_model = FKModel(keypoint_joints).cuda()
    if os.path.exists(CKPT_PATH):
        fk_model.load_state_dict(torch.load(CKPT_PATH))
        fk_model.eval()
    else:
        print("❌ 모델 파일 없음")
        return

    # 4. 테스트 데이터 생성 및 정규화
    limits = robot.get_qlimits()
    # 랜덤 포즈 1000개 생성
    q_batch_raw = np.random.uniform(limits[:, 0], limits[:, 1], size=(NUM_SAMPLES, len(limits))).astype(np.float32)
    
    # ★ 정규화 (Normalization) 로직 점검 ★
    # 학습 시 [-1, 1]로 정규화했다고 가정
    q_min = limits[:, 0]
    q_max = limits[:, 1]
    q_batch_norm = 2 * (q_batch_raw - q_min) / (q_max - q_min) - 1
    
    q_tensor = torch.from_numpy(q_batch_norm).float().cuda()

    # 5. 예측 실행
    with torch.no_grad():
        # Output: [Batch, 5, 3]
        pred_positions = fk_model(q_tensor).cpu().numpy()

    # 6. 정답(GT) 추출 및 오차 계산
    errors = [] # 전체 오차 저장
    per_finger_errors = [[] for _ in range(5)] # 손가락별 오차 저장
    
    # (선택적 안전장치) 혹시 모를 inf 값이 있는지 체크
    if np.isinf(limits).any():
        print("⚠️ [경고] URDF joint limit에 무한대(inf)가 포함되어 있어 랜덤 포즈 생성이 부정확할 수 있습니다.")

    for i in range(NUM_SAMPLES):
        robot.set_qpos(q_batch_raw[i]) # 로봇에는 Raw값 입력
        
        # [수정된 부분] SAPIEN 엔진에 관절값이 변경되었음을 알리고 3D 위치를 갱신합니다.
        if hasattr(robot, 'compute_forward_kinematics'):
            robot.compute_forward_kinematics() # SAPIEN 최신 버전에 따른 안전장치
        
        for f_idx, link in enumerate(target_links):
            gt_pos = link.get_pose().p # (3,)
            pred_pos = pred_positions[i, f_idx] # (3,)
            
            # 유클리드 거리 (L2 Norm) 계산
            dist = np.linalg.norm(gt_pos - pred_pos)
            
            errors.append(dist)
            per_finger_errors[f_idx].append(dist)

    # 7. 결과 리포트
    print("\n" + "="*40)
    print("📊 최종 검증 리포트 (단위: 미터)")
    print("="*40)
    
    mean_error = np.mean(errors)
    max_error = np.max(errors)
    
    print(f"✅ 전체 평균 오차: {mean_error*1000:.2f} mm")
    print(f"⚠️ 최대 발생 오차: {max_error*1000:.2f} mm")
    
    print("\n🖐️  손가락별 평균 오차:")
    for i, f_name in enumerate(finger_names):
        f_mean = np.mean(per_finger_errors[i])
        print(f"   - {f_name}: {f_mean*1000:.2f} mm")
        
    print("-" * 40)
    
    # 8. 판정
    if mean_error < 0.01: # 1cm 미만
        print("🎉 [PASS] 모델 신뢰성 확보됨. (IK 학습 진행 가능)")
    elif mean_error < 0.03: # 3cm 미만
        print("⚠️ [WARNING] 오차가 다소 큼. 데이터 정규화 방식 재확인 필요.")
    else:
        print("❌ [FAIL] 모델이나 매핑에 심각한 오류가 있음.")

if __name__ == "__main__":
    verify_precision()