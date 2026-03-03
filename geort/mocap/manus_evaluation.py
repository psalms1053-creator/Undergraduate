# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

from unittest import loader
from geort.mocap.manus_mocap import ManusMocap
from geort.env.hand import HandKinematicModel
from geort import load_model, get_config
import argparse

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

    for link in hand.hand.get_links():
        for shape in link.get_collision_shapes():
            # (group0, group1, group2, group3) -> 모두 0으로 설정하여 충돌 무시
            shape.set_collision_groups(0, 0, 0, 0)
    
    # Run!
    while True:
        viewer_env.update()

        result = mocap.get()

        if result['status'] == 'recording' and result["result"] is not None:
            qpos = model.forward(result["result"])
            hand.set_qpos_target(qpos)

        if result['status'] == 'quit':
            break 

if __name__ == '__main__':
    main()
