import torch
import numpy as np
import sapien.core as sapien
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geort.model import FKModel, IKModel
import os

# ================= 사용자 설정 =================
base_path = "/home/jy/ros2_ws/src/GeoRT"
HUMAN_DATA_PATH = os.path.join(base_path, "data/0307_2.npy")
URDF_PATH = os.path.join(base_path, "assets/ASSY_Hand_R/test_fixed.urdf")

FK_CKPT_PATH = os.path.join(base_path, "checkpoint/fk_model_aidin_right_test.pth")
IK_CKPT_PATH = os.path.join(base_path, "checkpoint/aidin_right_test_last/last.pth")

# [수정 1] get_config 대신 파일 내부에 직접 설정 정의
robot_config = {
    "fingertip_link": [
        {"name": "thumb",  "center_offset": [0.024, 0.0, 0.0]},
        {"name": "index",  "center_offset": [0.0, 0.013, 0.0]},
        {"name": "middle", "center_offset": [0.0, 0.013, 0.0]},
        {"name": "ring",   "center_offset": [0.0, 0.013, 0.0]},
        {"name": "baby",   "center_offset": [0.0, 0.013, 0.0]}
    ]
}

NUM_BG_SAMPLES = 300 
NUM_IK_SAMPLES = 70   
# ==============================================

def visualize_complete():
    print("📊 [종합 시각화] FK(GT/Pred) + Human + IK Result 로딩 중...")
    
    # 1. 로봇 및 엔진 초기화
    engine = sapien.Engine()
    scene = engine.create_scene()
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    try:
        robot = loader.load(URDF_PATH)
    except Exception as e:
        print(f"❌ URDF 로드 실패: {e}")
        return

    # Joint & Links 설정
    keypoint_joints = [[0,1,2,3], [4,5,6,7], [8,9,10,11], [12,13,14,15], [16,17,18,19]]
    active_joints = robot.get_active_joints()
    tip_links = [active_joints[g[-1]].get_child_link() for g in keypoint_joints]
    limits = robot.get_qlimits()

    # 2. 모델 로드 (FK & IK)
    fk_model = FKModel(keypoint_joints).cuda()
    if os.path.exists(FK_CKPT_PATH):
        fk_model.load_state_dict(torch.load(FK_CKPT_PATH, weights_only=True))
        fk_model.eval()
        print("✅ FK 모델 로드 완료")
    else:
        print("⚠️ FK 모델 없음 (경로 확인)")
    
    ik_model = IKModel(keypoint_joints).cuda()
    if os.path.exists(IK_CKPT_PATH):
        ik_model.load_state_dict(torch.load(IK_CKPT_PATH, weights_only=True))
        ik_model.eval()
        print("✅ IK 모델 로드 완료")
    else:
        print("⚠️ IK 모델 없음 (경로 확인)")

    # -------------------------------------------------------------------------
    # PART A: 🔵 파란점 (Robot GT) & 🔴 빨간점 (FK Pred)
    # -------------------------------------------------------------------------
    q_raw = np.random.uniform(limits[:,0], limits[:,1], size=(NUM_BG_SAMPLES, len(limits))).astype(np.float32)
    q_norm = 2 * (q_raw - limits[:,0]) / (limits[:,1] - limits[:,0]) - 1

    with torch.no_grad():
        fk_pred_tips = fk_model(torch.from_numpy(q_norm).float().cuda()).cpu().numpy().reshape(-1, 3)

    fk_gt_tips = []
    for q in q_raw:
        robot.set_qpos(q)
        for i, link in enumerate(tip_links):
            offset = robot_config['fingertip_link'][i]['center_offset']
            p_link = link.get_pose()
            p_tip = p_link.to_transformation_matrix() @ np.array([*offset, 1.0])
            fk_gt_tips.append(p_tip[:3])
    fk_gt_tips = np.array(fk_gt_tips)

    # -------------------------------------------------------------------------
    # PART B: 🟢 초록점 (Human Target) & 🟠 주황/노란점 (IK Result)
    # -------------------------------------------------------------------------
    human_targets = []
    ik_results = []

    if os.path.exists(HUMAN_DATA_PATH):
        data = np.load(HUMAN_DATA_PATH, allow_pickle=True)
        if len(data.shape) == 2 and data.shape[1] == 63:
            data = data.reshape(-1, 21, 3)
        
        if len(data) > NUM_IK_SAMPLES:
            indices = np.random.choice(len(data), NUM_IK_SAMPLES, replace=False)
            data = data[indices]

        human_targets = data[:, [4, 8, 12, 16, 20], :] # (B, 5, 3)
        
        with torch.no_grad():
            inp = torch.from_numpy(human_targets).float().cuda()
            pred_q_norm = ik_model(inp).cpu().numpy()
        
        pred_q_real = (pred_q_norm + 1) * (limits[:,1] - limits[:,0]) / 2 + limits[:,0]
        
        for q in pred_q_real:
            robot.set_qpos(q)
            current_frame_tips = []
            for i, link in enumerate(tip_links):
                # [수정 2] IK 결과물에도 center_offset을 적용하여 정확한 손끝 좌표 계산!
                offset = robot_config['fingertip_link'][i]['center_offset']
                p_link = link.get_pose()
                p_tip = p_link.to_transformation_matrix() @ np.array([*offset, 1.0])
                current_frame_tips.append(p_tip[:3])
            ik_results.append(current_frame_tips)
        ik_results = np.array(ik_results) 
    
    # -------------------------------------------------------------------------
    # PART C: 시각화
    # -------------------------------------------------------------------------
    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(111, projection='3d')

    # 1. 🔵 Robot GT (파란 점)
    ax.scatter(fk_gt_tips[:,0], fk_gt_tips[:,1], fk_gt_tips[:,2], 
               c='blue', s=5, alpha=0.2, label='Robot Workspace (GT)')
    
    # 2. 🔴 FK AI Pred (빨간 X)
    ax.scatter(fk_pred_tips[:,0], fk_pred_tips[:,1], fk_pred_tips[:,2], 
               c='red', marker='x', s=10, alpha=0.2, label='FK Prediction')

    # 3. 🟢 Human Target (초록 동그라미)
    if len(human_targets) > 0:
        h_flat = human_targets.reshape(-1, 3)
        ax.scatter(h_flat[:,0], h_flat[:,1], h_flat[:,2], 
                   c='green', marker='o', s=20, alpha=0.3, edgecolors='black', label='Human Target')
    
    # 4. 🟠 IK Result (주황 별 - 로봇이 실제로 수행한 결과)
    if len(ik_results) > 0:
        ik_flat = ik_results.reshape(-1, 3)
        ax.scatter(ik_flat[:,0], ik_flat[:,1], ik_flat[:,2], 
                   c='orange', marker='*', s=40, label='IK Result (Robot)')

        # 5. [연결선] Error Lines (Human <-> IK Result)
        for i in range(len(h_flat)):
            ax.plot([h_flat[i,0], ik_flat[i,0]],
                    [h_flat[i,1], ik_flat[i,1]],
                    [h_flat[i,2], ik_flat[i,2]],
                    color='gray', alpha=0.3, linewidth=1)

    ax.set_title("Comprehensive Check: Human(Green) -> IK -> Robot(Orange)")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    try: ax.set_box_aspect([1.2, 1, 1])
    except: pass

    ax.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    visualize_complete()