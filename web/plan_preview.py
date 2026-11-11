"""Read-only Nav2 path previews. Each browser request owns its action goal."""
import math
import threading
import uuid

from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient


class PlanPreview:
    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        self.lock = threading.Lock()
        self.active = {}

    def cancel(self, key):
        with self.lock:
            entry = self.active.get(key)
            if entry:
                entry['cancelled'].set()
                if entry['handle']:
                    entry['handle'].cancel_goal_async()
        return 200, {'cancelled': True}

    def run(self, body):
        key = str(uuid.UUID(body['id']))
        x, y, yaw = (body[k] for k in ('x', 'y', 'yaw'))
        if not all(type(v) in (int, float) and math.isfinite(v) for v in (x, y, yaw)):
            return 400, {'error': '目标坐标无效'}
        with self.node.lock:
            meta = self.node.maps.meta
            if not meta or meta['source'] != 'live' or self.node.robot_map is None:
                return 409, {'error': '等待实时地图和机器人定位'}
            if body.get('revision') != meta['revision']:
                return 409, {'error': '地图已更新，请重新确认目标'}
            ox, oy, angle = meta['origin']
            dx, dy = x - ox, y - oy
            c, s = math.cos(angle), math.sin(angle)
            col = math.floor((c*dx + s*dy)/meta['resolution'])
            row = math.floor((-s*dx + c*dy)/meta['resolution'])
            if not (0 <= col < meta['width'] and 0 <= row < meta['height']):
                return 422, {'error': '目标超出地图范围'}
            occupancy = self.node.grid[row*meta['width'] + col]
            if occupancy < 0 or occupancy >= 65:
                return 422, {'error': '目标位于未知区域或障碍物内，请重新选点'}
        if not self.client.server_is_ready():
            return 503, {'error': 'Nav2 规划器尚未就绪'}
        goal = ComputePathToPose.Goal()
        goal.planner_id = 'GridBased'
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x, goal.goal.pose.position.y = float(x), float(y)
        goal.goal.pose.orientation.z = math.sin(yaw / 2)
        goal.goal.pose.orientation.w = math.cos(yaw / 2)
        entry = {'handle': None, 'cancelled': threading.Event()}
        with self.lock:
            if self.active:
                return 409, {'error': '规划器正在计算，请稍后再试'}
            self.active[key] = entry
        try:
            accepted = threading.Event()
            def on_accept(future):
                try:
                    handle = future.result()
                    if handle.accepted:
                        with self.lock:
                            entry['handle'] = handle
                            if entry['cancelled'].is_set():
                                handle.cancel_goal_async()
                finally:
                    accepted.set()
            self.client.send_goal_async(goal).add_done_callback(on_accept)
            if not accepted.wait(5):
                self.cancel(key)
                return 504, {'error': '规划器响应超时'}
            if entry['cancelled'].is_set() or not entry['handle']:
                return 409, {'error': '规划已取消或被拒绝'}
            finished = threading.Event()
            future = entry['handle'].get_result_async()
            future.add_done_callback(lambda _: finished.set())
            if not finished.wait(12):
                self.cancel(key)
                return 504, {'error': '计算超时，请重新选点'}
            result = future.result()
            if entry['cancelled'].is_set() or result.status == GoalStatus.STATUS_CANCELED:
                return 409, {'error': '规划已取消'}
            path = [[p.pose.position.x, p.pose.position.y] for p in result.result.path.poses]
            if result.status != GoalStatus.STATUS_SUCCEEDED or not path:
                return 422, {'error': '未找到可通行路径，请调整目标位置'}
            ticket = self.node.drive.remember(result.result.path, body['revision']) if self.node.drive else None
            return 200, {'ticket':ticket, 'path': path, 'length': sum(math.dist(a,b) for a,b in zip(path,path[1:]))}
        finally:
            with self.lock:
                self.active.pop(key, None)
