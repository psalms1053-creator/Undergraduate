import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import numpy as np
from scipy.spatial.transform import Rotation as R

# ---------------------------------------------------------
# [주의] 이 부분을 이전에 확인하신 메시지 타입으로 수정해 주세요!
# 예: from manus_msgs.msg import GloveState 
from manus_ros2_msgs.msg import ManusGlove 
# ---------------------------------------------------------

class ManusRealtimeVizNode(Node):
    def __init__(self):
        super().__init__('manus_realtime_viz_node')
        
        self.subscription = self.create_subscription(
            ManusGlove,
            '/manus_glove_1',
            self.listener_callback,
            10
        )
        
        # 최신 데이터를 저장할 변수 (초기값은 비어있음)
        self.latest_nodes = None
        self.get_logger().info('ROS 2 노드 시작: 데이터를 기다리는 중...')

    def listener_callback(self, msg):
        # 메시지가 들어오면 파이썬 딕셔너리 리스트로 변환하여 저장
        parsed_nodes = []
        for n in msg.raw_nodes:
            parsed_nodes.append({
                "node_id": n.node_id,
                "parent_node_id": n.parent_node_id,
                "joint_type": n.joint_type,
                "chain_type": n.chain_type,
                "pose": {
                    "position": {"x": n.pose.position.x, "y": n.pose.position.y, "z": n.pose.position.z},
                    "orientation": {"x": n.pose.orientation.x, "y": n.pose.orientation.y, "z": n.pose.orientation.z, "w": n.pose.orientation.w}
                }
            })
        self.latest_nodes = parsed_nodes

class RealtimePlotter:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        
        # 그래프 초기화
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.view_init(elev=20, azim=135)
        
        self.chain_colors = {
            "Hand":   "#2C3E50", "Thumb":  "#E74C3C", "Index":  "#F1C40F",
            "Middle": "#2ECC71", "Ring":   "#3498DB", "Pinky":  "#9B59B6"
        }

    def update_plot(self, frame):
        # ROS 2 이벤트를 한 번씩 처리해서 새 데이터를 받아옴
        rclpy.spin_once(self.ros_node, timeout_sec=0.01)
        
        data = self.ros_node.latest_nodes
        if not data:
            return # 데이터가 아직 안 들어왔으면 스킵
        
        self.ax.cla() # 이전 프레임 지우기
        nodes = {d['node_id']: d for d in data}
        
        # 범례 설정 (빈 플롯을 그려서 legend에 등록)
        for name, color in self.chain_colors.items():
            self.ax.plot([0], [0], [0], color=color, label=name, linewidth=2)

        # 노드 및 뼈대 그리기
        for node_id, node in nodes.items():
            pos = node['pose']['position']
            # 네가 작성한 좌표 변환 로직 반영
            px, py, pz = -pos['y'], -pos['x'], pos['z']
            ctype = node.get('chain_type', 'Hand')
            jtype = node.get('joint_type', '')
            line_color = self.chain_colors.get(ctype, 'black')
            
            # 노드 포인트
            self.ax.scatter(px, py, pz, color=line_color, s=30, alpha=0.9)
            
            # 텍스트 라벨 (옵션: 화면이 너무 복잡하면 주석 처리해도 됨)
            # label = f"{node_id}:{jtype[:1]}" if jtype != "Invalid" else f"{node_id}"
            # self.ax.text(px, py, pz + 0.003, label, fontsize=6, alpha=0.7)
            
            # 뼈대 연결
            pid = node['parent_node_id']
            if pid != node_id and pid in nodes:
                ppos = nodes[pid]['pose']['position']
                ppx, ppy, ppz = -ppos['y'], -ppos['x'], ppos['z']
                self.ax.plot([ppx, px], [ppy, py], [ppz, pz], color=line_color, linewidth=2, alpha=0.6)

        # Orientation 화살표 (연산량을 줄이려면 생략하거나 끝마디(TIP)만 그리는 것을 추천)
        arrow_len = 0.008
        for node_id, node in nodes.items():
            pos = node['pose']['position']
            ori = node['pose']['orientation']
            rot = R.from_quat([ori['x'], ori['y'], ori['z'], ori['w']])
            matrix = rot.as_matrix()
            
            for i, color in enumerate(['r', 'g', 'b']):
                dx, dy, dz = matrix[:, i] * arrow_len
                # 뼈대와 동일한 좌표 변환 적용 (-y, -x, z)
                self.ax.quiver(-pos['y'], -pos['x'], pos['z'], -dy, -dx, dz, color=color, linewidth=1, alpha=0.4)

        # 축 설정 고정 (화면이 덜덜 떨리는 것을 방지)
        self.ax.set_xlim([-0.1, 0.1])
        self.ax.set_ylim([-0.1, 0.1])
        self.ax.set_zlim([0.0, 0.2])
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('Manus Glove Realtime 3D', fontsize=15)
        self.ax.legend(loc='upper left', fontsize=8)

def main(args=None):
    rclpy.init(args=args)
    ros_node = ManusRealtimeVizNode()
    plotter = RealtimePlotter(ros_node)
    
    # FuncAnimation을 사용해 update_plot 함수를 50ms(약 20fps)마다 호출
    ani = animation.FuncAnimation(plotter.fig, plotter.update_plot, interval=50, cache_frame_data=False)
    
    try:
        # plt.show()가 메인 스레드를 잡고 있으면서 루프를 돕니다.
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()