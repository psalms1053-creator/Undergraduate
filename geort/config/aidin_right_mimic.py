{
    "name": "aidin_right_mimic",
    "urdf_path": "./assets/ASSY_Hand_R/test_fixed_delmic.urdf",
    "base_link": "base_link",
    "joint_order": [
        "joint1_thumb_z", "joint2_thumb_z", "joint3_thumb_z", "joint4_thumb_z",
        "joint1_index_z", "joint2_index_z", "joint3_index_z", "joint4_index_z",
        "joint1_middle_z", "joint2_middle_z", "joint3_middle_z", "joint4_middle_z",
        "joint1_ring_z", "joint2_ring_z", "joint3_ring_z", "joint4_ring_z",
        "joint1_baby_z", "joint2_baby_z", "joint3_baby_z", "joint4_baby_z"
    ],
    "fingertip_link": [
        {
            "name": "thumb",
            "link": "link4_thumb",
            "joint": ["joint1_thumb_z", "joint2_thumb_z", "joint3_thumb_z", "joint4_thumb_z"],
            "center_offset": [0.024, 0.0, 0.0],
            "human_hand_id": 4
        },
        {
            "name": "index",
            "link": "link4_index",
            "joint": ["joint1_index_z", "joint2_index_z", "joint3_index_z", "joint4_index_z"],
            "center_offset": [0.0, 0.013, 0.0],
            "human_hand_id": 8
        },
        {
            "name": "middle",
            "link": "link4_middle",
            "joint": ["joint1_middle_z", "joint2_middle_z", "joint3_middle_z", "joint4_middle_z"],
            "center_offset": [0.0, 0.013, 0.0],
            "human_hand_id": 12
        },
        {
            "name": "ring",
            "link": "link4_ring",
            "joint": ["joint1_ring_z", "joint2_ring_z", "joint3_ring_z", "joint4_ring_z"],
            "center_offset": [0.0, 0.013, 0.0],
            "human_hand_id": 16
        },
        {
            "name": "baby",
            "link": "link4_baby",
            "joint": ["joint1_baby_z", "joint2_baby_z", "joint3_baby_z", "joint4_baby_z"],
            "center_offset": [0.0, 0.013, 0.0],
            "human_hand_id": 20
        }
    ]
}
