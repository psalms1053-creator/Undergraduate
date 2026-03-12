# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

from unittest import loader
from geort.mocap.manus_mocap import ManusMocap
from geort.env.hand import HandKinematicModel
from geort import load_model, get_config
import argparse
import numpy as np

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-hand', type=str, default='allegro')
    parser.add_argument('-ckpt_tag', type=str, default='alex')  # Your CKPT Tag.

    args = parser.parse_args()

    # GeoRT Model.
    model = load_model(args.ckpt_tag)

    
    # Motion Capture.
    mocap = ManusMocap()
    
    # Robot Simulation.
    config = get_config(args.hand)
    hand = HandKinematicModel.build_from_config(config, render=True)
    viewer_env = hand.get_viewer_env()
    '''
    for link in hand.hand.get_links():
        for shape in link.get_collision_shapes():
            # (group0, group1, group2, group3) -> 모두 0으로 설정하여 충돌 무시
            shape.set_collision_groups(0, 0, 0, 0)
    '''

    #'''
    # Run!
    while True:
        viewer_env.update()

        result = mocap.get()

        if result['status'] == 'recording' and result["result"] is not None:
            qpos = model.forward(result["result"])

            hand.set_qpos_target(qpos)

        if result['status'] == 'quit':
            break 
    '''
    while True:
        viewer_env.update() # SAPIEN 물리 엔진 1스텝 진행

        result = mocap.get()

        if result['status'] == 'recording' and result["result"] is not None:
            # 1. 모델이 예측한 타겟 각도 (이미 Limit 내에 보장된 값)
            qpos_target = model.forward(result["result"])

            if hasattr(hand, 'joint_limits'):
                lower_limits = hand.joint_limits[:, 0]
                upper_limits = hand.joint_limits[:, 1]
                qpos_target = np.clip(qpos_target, lower_limits, upper_limits)
            
            # 2. 로봇 제어기에 타겟값 전달
            hand.set_qpos_target(qpos_target)

            # 3. SAPIEN 로봇의 '현재 실제' 관절 각도 가져오기
            # (hand.py 내부 구조에 따라 SAPIEN의 get_qpos를 호출합니다)
            if hasattr(hand, 'hand'):
                qpos_actual = hand.hand.get_qpos()[:len(qpos_target)]
            else:
                qpos_actual = np.zeros_like(qpos_target)

            # 4. 실시간 터미널 출력 
            print(qpos_target-qpos_actual,end='\r')

        if result['status'] == 'quit':
            print("\n[시스템 종료] 시뮬레이션을 마칩니다.") # 덮어쓰기 방지용 줄바꿈
            break
        #'''

if __name__ == '__main__':
    main()
