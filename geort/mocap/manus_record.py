from __future__ import annotations

import argparse
import select
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import zmq

import geort


@dataclass
class RecConfig:
    name: str
    host: str = "127.0.0.1"
    port: int = 8765
    target_hz: float = 30.0
    print_every: int = 30  # frames
    max_wait_s: float = 10.0


def _setup_terminal_raw_mode() -> Optional[Tuple[int, object]]:
    """
    Linux 터미널에서 엔터 없이 단일 키 입력(s/e/q)을 받기 위해 raw mode 설정.
    실패하면 None (그 경우 '엔터'가 필요할 수 있음).
    """
    try:
        import termios
        import tty

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)  # 1-char
        return fd, old
    except Exception:
        return None


def _restore_terminal(fd_old: Optional[Tuple[int, object]]) -> None:
    if not fd_old:
        return
    try:
        import termios

        fd, old = fd_old
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    except Exception:
        pass


def _poll_key_nonblocking() -> Optional[str]:
    """
    stdin에서 키 1개를 non-blocking으로 읽음. 없으면 None.
    """
    try:
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        if r:
            ch = sys.stdin.read(1)
            return ch
    except Exception:
        return None
    return None


def _connect_zmq_sub(host: str, port: int) -> zmq.Socket:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.connect(f"tcp://{host}:{port}")
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    sock.setsockopt(zmq.RCVHWM, 1)
    return sock


def _try_recv_hand21(sock: zmq.Socket) -> Optional[np.ndarray]:
    """
    ZMQ 메시지: float32 bytes, shape=(21,3) 기대.
    """
    try:
        msg = sock.recv(flags=zmq.NOBLOCK)
    except zmq.Again:
        return None

    arr = np.frombuffer(msg, dtype=np.float32)
    if arr.size != 63:
        return None
    return arr.reshape(21, 3).copy()


def _sleep_to_rate(last_t: float, hz: float) -> float:
    if hz <= 0:
        return time.time()
    dt = 1.0 / hz
    now = time.time()
    sleep_t = dt - (now - last_t)
    if sleep_t > 0:
        time.sleep(sleep_t)
    return time.time()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", type=str, required=True, help="saved as data/<name>.npy")
    parser.add_argument("--host", type=str, default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--target_hz", type=float, default=30.0)
    parser.add_argument("--print_every", type=int, default=30)
    parser.add_argument("--max_wait_s", type=float, default=10.0)
    args = parser.parse_args()

    cfg = RecConfig(
        name=args.name,
        host=args.host,
        port=args.port,
        target_hz=args.target_hz,
        print_every=args.print_every,
        max_wait_s=args.max_wait_s,
    )

    sock = _connect_zmq_sub(cfg.host, cfg.port)

    print(f"[manus_record] ZMQ SUB connect tcp://{cfg.host}:{cfg.port}")
    print("[manus_record] keys: s=start, e=pause, q=save&quit, Ctrl+C=save&quit")
    print("[manus_record] waiting first frame...")

    # 터미널 raw mode
    raw_state = _setup_terminal_raw_mode()

    frames: List[np.ndarray] = []
    status = "idle"  # idle / recording / pause

    # first frame wait (연결 확인)
    t0 = time.time()
    while True:
        kp = _try_recv_hand21(sock)
        if kp is not None:
            print(f"[manus_record] first frame ok. wrist={kp[0].tolist()}")
            break
        if time.time() - t0 > cfg.max_wait_s:
            _restore_terminal(raw_state)
            raise RuntimeError(
                f"No data within {cfg.max_wait_s}s from tcp://{cfg.host}:{cfg.port}. "
                "Check manus_mocap_core (publisher) and port."
            )
        time.sleep(0.01)

    last_tick = time.time()

    try:
        while True:
            last_tick = _sleep_to_rate(last_tick, cfg.target_hz)

            ch = _poll_key_nonblocking()
            if ch:
                ch = ch.lower()
                if ch == "s":
                    status = "recording"
                    print("[manus_record] status=recording")
                elif ch == "e":
                    status = "pause"
                    print("[manus_record] status=pause")
                elif ch == "q":
                    print("[manus_record] status=quit")
                    break

            kp = _try_recv_hand21(sock)

            if status == "recording" and kp is not None:
                frames.append(kp)
                n = len(frames)
                if n % max(cfg.print_every, 1) == 0:
                    print(f"Data collected: {n}")

    except KeyboardInterrupt:
        print("\n[manus_record] Ctrl+C received -> saving...")

    finally:
        _restore_terminal(raw_state)
        sock.close()

    if len(frames) == 0:
        print("[manus_record] collected 0 frames. not saving.")
        return

    arr = np.asarray(frames, dtype=np.float32)
    save_path = geort.save_human_data(arr, cfg.name)
    print("Data saved to", save_path)
    print(f"[manus_record] shape={arr.shape} dtype={arr.dtype}")


if __name__ == "__main__":
    main()
