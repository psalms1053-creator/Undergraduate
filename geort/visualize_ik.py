import torch
import numpy as np
import sapien.core as sapien
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geort.model import IKModel, FKModel  # IKModel 필요
import os

# ================= 사용자 설정 =================
base_path = "/home/jy/ros2_ws/src/GeoRT"
HUMAN_DATA_PATH = os.path.join(base_path, "data/AIDIN_TEST_0211.npy")
URDF_PATH = os.path.join(base_path, "assets/ASSY_Hand_R/test_fixed.urdf")

# ⚠️ IK 모델 체크포인트 경로를 정확히 수정해주세요! (예: epoch_100.pth 또는 last.pth)
# /home/jy/ros2_ws/src/GeoRT/checkpoint/aidin_right_test_last/last.pth
IK_CKPT_PATH = os.path.join(base_path, "checkpoint/aidin_right_test_last/last.pth") 

NUM_SAMPLES = 250  # 너무 많으면 선이 복잡하니 적당히 설정
# ==============================================

def visualize_ik_performance():
    print("🎯 [IK Check] Human Target vs Robot Result 비교 시작...")

    # 1. 로봇 로드
    engine = sapien.Engine()
    scene = engine.create_scene()
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    try:
        robot = loader.load(URDF_PATH)
    except Exception as e:
        print(f"❌ URDF 로드 실패: {e}")
        return

    # Joint 그룹 정의 (5개 손가락)
    keypoint_joints = [[0,1,2,3], [4,5,6,7], [8,9,10,11], [12,13,14,15], [16,17,18,19]]
    active_joints = robot.get_active_joints()
    tip_links = [active_joints[g[-1]].get_child_link() for g in keypoint_joints]

    # 2. IK 모델 로드
    ik_model = IKModel(keypoint_joints).cuda()
    if os.path.exists(IK_CKPT_PATH):
        print(f"✅ IK 체크포인트 로드: {IK_CKPT_PATH}")
        ik_model.load_state_dict(torch.load(IK_CKPT_PATH))
        ik_model.eval()
    else:
        print(f"⚠️ IK 체크포인트를 찾을 수 없습니다: {IK_CKPT_PATH}")
        print("    -> 경로를 확인하거나 학습이 되었는지 확인하세요.")
        return

    # 3. 사람 데이터(Target) 준비
    target_tips = []
    if os.path.exists(HUMAN_DATA_PATH):
        try:
            data = np.load(HUMAN_DATA_PATH, allow_pickle=True)
            if len(data.shape) == 2 and data.shape[1] == 63:
                data = data.reshape(-1, 21, 3)
            
            # 랜덤 샘플링
            if len(data) > NUM_SAMPLES:
                indices = np.random.choice(len(data), NUM_SAMPLES, replace=False)
                data = data[indices]
            
            # 손끝(Tip)만 추출: 인덱스 [4, 8, 12, 16, 20]
            # IK 모델 입력 형태: (B, 5, 3)
            target_tips = data[:, [4, 8, 12, 16, 20], :]
            print(f"📂 Target Data: {target_tips.shape}")

        except Exception as e:
            print(f"❌ 데이터 로드 에러: {e}")
            return
    else:
        print("⚠️ 사람 데이터 파일 없음")
        return

    # 4. IK 추론 (Target -> Joint Angles)
    with torch.no_grad():
        inputs = torch.from_numpy(target_tips).float().cuda() # (B, 5, 3)
        pred_q_norm = ik_model(inputs).cpu().numpy()          # (B, 20) - Normalized joints

    # 5. 결과 검증 (Joint Angles -> Actual Robot Tips)
    #    FK 모델을 따로 쓰지 않고, 정확한 검증을 위해 로봇(Sapien)을 직접 움직여 봅니다.
    limits = robot.get_qlimits()
    # Denormalize: [-1, 1] -> [min, max]
    pred_q_real = (pred_q_norm + 1) * (limits[:,1] - limits[:,0]) / 2 + limits[:,0]
    
    robot_result_tips = []
    for q in pred_q_real:
        robot.set_qpos(q)
        tips = [link.get_pose().p for link in tip_links]
        robot_result_tips.append(tips)
    robot_result_tips = np.array(robot_result_tips) # (B, 5, 3)

    # 6. 시각화 (Error Line 그리기)
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Flatten for plotting
    target_flat = target_tips.reshape(-1, 3)
    result_flat = robot_result_tips.reshape(-1, 3)

    # (1) 목표 위치 (Human/Target) - 초록색
    ax.scatter(target_flat[:,0], target_flat[:,1], target_flat[:,2], 
               c='green', s=20, label='Target (Human)')

    # (2) 결과 위치 (Robot Result) - 빨간색
    ax.scatter(result_flat[:,0], result_flat[:,1], result_flat[:,2], 
               c='red', marker='x', s=20, label='Result (IK)')

    # (3) 오차 연결선 (Error Lines)
    # 목표와 결과 사이를 선으로 연결합니다. 선이 길수록 오차가 큰 것입니다.
    for i in range(len(target_flat)):
        ax.plot([target_flat[i,0], result_flat[i,0]],
                [target_flat[i,1], result_flat[i,1]],
                [target_flat[i,2], result_flat[i,2]],
                color='gray', alpha=0.3, linewidth=1)

    ax.set_title(f"IK Performance Check (Lines = Error)")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    
    try: ax.set_box_aspect([1, 1, 1])
    except: pass

    plt.legend()
    plt.show()

if __name__ == "__main__":
    visualize_ik_performance()