import torch
import numpy as np
import sapien.core as sapien
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geort.model import FKModel
import os

# ================= 사용자 설정 =================
# 경로 설정 (사용자 환경에 맞게 수정됨)
base_path = "/home/jy/ros2_ws/src/GeoRT"
URDF_PATH = os.path.join(base_path, "assets/ASSY_Hand_R/test_fixed.urdf")
CKPT_PATH = os.path.join(base_path, "checkpoint/fk_model_aidin_right_test.pth")

# 샘플 개수 (많아도 점만 찍으면 빠릅니다)
NUM_ROBOT_SAMPLES = 1000
# ==============================================

def visualize_fast():
    print("🚀 [Fast Mode] 점 데이터 고속 시각화 시작...")

    # 1. 로봇 설정 및 FK 모델 로드
    engine = sapien.Engine()
    scene = engine.create_scene()
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    try:
        robot = loader.load(URDF_PATH)
    except Exception as e:
        print(f"❌ URDF 로드 실패: {e}")
        return

    # FK 모델 준비
    keypoint_joints = [[0,1,2,3], [4,5,6,7], [8,9,10,11], [12,13,14,15], [16,17,18,19]]
    active_joints = robot.get_active_joints()
    tip_links = [active_joints[g[-1]].get_child_link() for g in keypoint_joints]

    fk_model = FKModel(keypoint_joints).cuda()
    if os.path.exists(CKPT_PATH):
        fk_model.load_state_dict(torch.load(CKPT_PATH))
        fk_model.eval()
    
    # 2. 로봇 데이터 생성 (Batch Processing)
    limits = robot.get_qlimits()
    # 랜덤 관절 각도 생성
    q_raw = np.random.uniform(limits[:,0], limits[:,1], size=(NUM_ROBOT_SAMPLES, len(limits))).astype(np.float32)
    q_norm = 2 * (q_raw - limits[:,0]) / (limits[:,1] - limits[:,0]) - 1
    
    # AI 예측 (FK Model)
    with torch.no_grad():
        pred_tips = fk_model(torch.from_numpy(q_norm).float().cuda()).cpu().numpy().reshape(-1, 3)

    # Ground Truth (Sapien Engine) - 여전히 for문이 필요하지만 로직 최소화
    gt_tips = []
    for q in q_raw:
        robot.set_qpos(q)
        # 5개 손끝 위치 한 번에 추출
        gt_tips.append([link.get_pose().p for link in tip_links])
    gt_tips = np.array(gt_tips).reshape(-1, 3)

    # 3. 시각화 (Scatter 한 번씩만 호출)
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    # [1] Robot GT (Blue)
    ax.scatter(gt_tips[:,0], gt_tips[:,1], gt_tips[:,2], 
               c='blue', s=5, alpha=0.15, label='Robot Workspace')

    # [2] Robot Pred (Red X)
    # 겹침 확인용으로 AI 예측값도 뿌려봅니다.
    ax.scatter(pred_tips[:,0], pred_tips[:,1], pred_tips[:,2], c='red', marker='x', s=10, alpha=0.15, label='AI Pred')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("Fast Check: Human vs Robot Workspace Overlay")
    
    # 축 비율
    try: ax.set_box_aspect([1,1,1])  # Z축을 좀 더 길게
    except: pass

    plt.legend()
    plt.show()

if __name__ == "__main__":
    visualize_fast()