# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.
#
# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

from __future__ import annotations

import argparse
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import zmq

try:
    from manus_ros2_msgs.msg import ManusGlove  # type: ignore
except Exception:
    ManusGlove = None  # type: ignore

def hand_to_canonical(hand_point: np.ndarray) -> np.ndarray:
    """
    Canonical frame (right-hand convention-ish):
      z = p9 - p0
      y_aux = p5 - p13
      x = cross(y_aux, z)
      y = cross(z, x)
    """
    hp = np.asarray(hand_point, dtype=np.float64)
    z_axis = hp[9] - hp[0]
    z_axis /= (np.linalg.norm(z_axis) + 1e-12)

    y_axis_aux = hp[5] - hp[13]
    y_axis_aux /= (np.linalg.norm(y_axis_aux) + 1e-12)

    x_axis = np.cross(y_axis_aux, z_axis)
    x_axis /= (np.linalg.norm(x_axis) + 1e-12)

    y_axis = np.cross(z_axis, x_axis)
    y_axis /= (np.linalg.norm(y_axis) + 1e-12)

    rotation_base = np.array([x_axis, y_axis, z_axis]).transpose()
    translation_base = hp[0]

    T = np.eye(4)
    T[:3, :3] = rotation_base
    T[:3, 3] = translation_base

    T_inv = np.linalg.inv(T)
    hp_h = np.concatenate([hp, np.ones((hp.shape[0], 1))], axis=-1)
    hp_can = hp_h @ T_inv.transpose()
    return hp_can[:, :3].astype(np.float32)


# /manus_glove_1 에서 0..24(25개)로 올 때, 5/10/15/20을 제거하면 21개가 됨.
GLOVE25_TO_21_KEEP = np.array(
    [0, 1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24],
    dtype=np.int64,
)

def extract_glove_points_25x3(msg) -> Optional[np.ndarray]:
    """
    manus_ros2_msgs/msg/ManusGlove 기준:
      msg.raw_nodes: ManusRawNode[]
      raw_node.node_id: int
      raw_node.pose.position: geometry_msgs/Point (x,y,z)

    수정 사항:
      1. 데이터 파싱
      2. Y값 부호 반전
      3. Z축 중심 시계방향(CW) 90도 회전 적용
         (x, y_inv) -> CW 90 -> (y_inv, -x) = (-y, -x)
    """
    if not hasattr(msg, "raw_nodes"):
        return None

    pts = np.full((25, 3), np.nan, dtype=np.float32)

    for rn in msg.raw_nodes:
        try:
            nid = int(rn.node_id)
        except Exception:
            continue

        if 0 <= nid < 25:
            p = rn.pose.position
            
            # 1. 원본 데이터 (x, y, z)
            raw_x = float(p.x)
            raw_y = float(p.y)
            raw_z = float(p.z)

            # 2. Y값 부호 반전 (y -> -y)
            # 3. Z축 시계방향 회전 적용
            #    공식: New_X = (Inverted_Y), New_Y = -(Raw_X)
            
            pts[nid, 0] = -raw_y  # 변환된 X
            pts[nid, 1] = -raw_x  # 변환된 Y
            pts[nid, 2] = raw_z   # Z는 유지

    if np.isnan(pts).any():
        return None

    return pts

def glove25_to_hand21(points25: np.ndarray) -> np.ndarray:
    """
    규칙: /manus_glove_1에서 5,10,15,20을 제거하고 21개로 만들기
    """
    return points25[GLOVE25_TO_21_KEEP]


class Manus(Node):
    """
    ROS2 SUB -> (21,3) -> (optional canonicalize) -> ZMQ PUB
    """
    def __init__(self, glove_topic: str, port: int, canonicalize: bool):
        super().__init__("manus_visualizer")

        if ManusGlove is None:
            raise RuntimeError(
                "Cannot import manus_ros2_msgs.msg.ManusGlove. "
                "Ensure manus_ros2_msgs is installed in this env."
            )

        self._canonicalize = bool(canonicalize)
        
        # EMA 관련 변수 제거됨
        
        self._latest_hand21: Optional[np.ndarray] = None
        self._lock = threading.Lock()

        self._sub = self.create_subscription(
            ManusGlove,
            glove_topic,
            self._on_glove,
            qos_profile_sensor_data,
        )

        self.port = int(port)
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 0)
        self.socket.bind(f"tcp://*:{self.port}")

        self.get_logger().info(f"[manus_core] SUB {glove_topic} -> ZMQ PUB tcp://*:{self.port}")
        self.get_logger().info(f"[manus_core] canonicalize={self._canonicalize} (EMA Disabled)")

    def _on_glove(self, msg) -> None:
        points25 = extract_glove_points_25x3(msg)
        if points25 is None:
            self.get_logger().warn("[manus_core] cannot extract (25,3) from /manus_glove_1")
            return

        hand21 = glove25_to_hand21(points25)

        if self._canonicalize:
            hand21 = hand_to_canonical(hand21)

        # EMA 필터링 로직 제거됨: 들어온 데이터 그대로 최신 데이터로 업데이트
        with self._lock:
            self._latest_hand21 = hand21

    def run(self, publish_hz: float = 60.0) -> None:
        dt = 1.0 / max(float(publish_hz), 1e-6)
        count = 0
        last_log = 0.0

        while rclpy.ok():
            t0 = time.time()
            with self._lock:
                hand21 = None if self._latest_hand21 is None else self._latest_hand21.copy()

            if hand21 is not None:
                self.socket.send(hand21.astype(np.float32).tobytes())
                count += 1

                now = time.time()
                if now - last_log > 1.0:
                    last_log = now
                    self.get_logger().info(
                        f"[manus_core] broadcasting #{count} shape={hand21.shape} wrist={hand21[0].tolist()}"
                    )

            sleep_t = dt - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", type=str, default="/manus_glove_1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--publish_hz", type=float, default=60.0)
    parser.add_argument("--no_canonicalize", action="store_true")
    # --ema_alpha 인자 제거됨
    cfg = parser.parse_args(args=args)

    rclpy.init(args=None)
    manus_node = Manus(
        glove_topic=cfg.topic,
        port=cfg.port,
        canonicalize=(not cfg.no_canonicalize),
        # ema_alpha 전달 제거됨
    )

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(manus_node)

    run_thread = threading.Thread(target=manus_node.run, kwargs={"publish_hz": cfg.publish_hz}, daemon=True)
    run_thread.start()

    try:
        executor.spin()
    finally:
        executor.shutdown()
        manus_node.destroy_node()
        rclpy.shutdown()
        run_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()