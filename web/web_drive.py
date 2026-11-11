"""Execute a confirmed preview through FollowPath and the collision monitor."""
import json
import math
from pathlib import Path
import threading
import time
from collections import OrderedDict
from secrets import token_urlsafe

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, TwistStamped
from hdas_msg.msg import ControllerSignalStamped, FeedbackStatus
from map_alignment import MapAlignment
from path_speed import PathSpeedProfile
from nav2_msgs.action import FollowPath
from nav2_msgs.srv import IsPathValid
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data


SAFE = '/r1pro_nav/nav2/cmd_vel_safe'
HARDWARE = '/motion_target/target_speed_chassis'


class WebDrive:
    def __init__(self, node):
        self.n = node
        config = Path(__file__).resolve().parents[1] / 'src/r1pro_waypoint_nav/config/navigation_profile.json'
        profile = json.loads(config.read_text())
        self.speed = float(profile['linear_speed'])
        self.yaw_speed = float(profile['yaw_speed'])
        if not all(math.isfinite(v) and v > 0 for v in (self.speed,self.yaw_speed)):
            raise ValueError('Invalid navigation speed limits')
        self.alignment = MapAlignment(node)
        self.speed_profile = None
        self.speed_limit_pub = node.create_publisher(SpeedLimit, '/speed_limit', 1)
        self.last_id = None
        self.lock = threading.RLock()
        self.paths = OrderedDict()
        self.goal = self.handle = self.pub = None
        self.phase, self.message = 'idle', '等待确认目标'
        self.stop_until = self.accepted_at = self.phase_since = 0.
        self.safe_at = self.feedback_at = self.mode_at = self.status_at = 0.
        self.mode = None
        self.fault = False
        self.command = None
        self.still = False
        self.client = ActionClient(node, FollowPath, '/follow_path')
        self.check = node.create_client(IsPathValid, '/is_path_valid')
        node.create_subscription(Twist, SAFE, self.safe, 1)
        node.create_subscription(TwistStamped, '/motion_control/chassis_speed', self.feedback, qos_profile_sensor_data)
        node.create_subscription(ControllerSignalStamped, '/controller', self.control, qos_profile_sensor_data)
        node.create_subscription(FeedbackStatus, '/hdas/feedback_status_chassis', self.status, qos_profile_sensor_data)
        node.create_timer(.05, self.tick)

    def control(self, msg):
        self.mode, self.mode_at = msg.data.mode, time.monotonic()

    def feedback(self, msg):
        age = self.n.get_clock().now().nanoseconds * 1e-9 - msg.header.stamp.sec - msg.header.stamp.nanosec * 1e-9
        if -.05 <= age <= .25:
            self.feedback_at = time.monotonic()
            self.still = math.hypot(msg.twist.linear.x, msg.twist.linear.y) < .015 and abs(msg.twist.angular.z) < .035

    def status(self, msg):
        self.fault = any(e.error_code for e in msg.errors)
        self.status_at = time.monotonic()

    def safe(self, msg):
        self.command, self.safe_at = msg, time.monotonic()

    def remember(self, path, revision):
        with self.lock:
            key = token_urlsafe(20)
            self.paths[key] = (path, revision)
            if len(self.paths) > 32:
                self.paths.popitem(last=False)
            return key

    def problem(self):
        now = time.monotonic()
        if self.mode != 5 or now-self.mode_at > .3:
            return '底盘未处于有效的外部控制模式'
        if self.fault or now-self.status_at > 2:
            return '底盘故障或电机反馈超时'
        if now-self.feedback_at > .25 or self.n.robot_map is None:
            return '缺少新鲜的底盘反馈或地图定位'
        if now-self.alignment.sample_time > 2 or self.alignment.score < .65:
            return '激光与地图定位不一致或激光已超时'
        if not self.client.server_is_ready() or not self.check.service_is_ready():
            return '路径跟踪服务尚未就绪'
        if self.n.count_publishers(HARDWARE) > int(self.pub is not None):
            return '另一个控制器正在发布底盘指令'
        if self.n.count_subscribers(HARDWARE) == 0:
            return '底盘未连接'
        sources = self.n.get_publishers_info_by_topic(SAFE)
        if len(sources) != 1 or sources[0].node_name != 'collision_monitor':
            return '避障速度输出未就绪'
        return ''

    def snapshot(self):
        with self.lock:
            reason = self.problem()
            return dict(max_linear_speed=self.speed, available_reason=reason, active=self.goal is not None, phase=self.phase, message=self.message,
                        ready=not reason and self.pub is None, id=self.goal or self.last_id, alignment=self.alignment.score)

    def start(self, key):
        with self.lock:
            if self.goal or self.pub:
                return 409, dict(error='导航或停车正在进行')
            if key not in self.paths:
                return 409, dict(error='请重新选点并预览路径')
            problem = self.problem()
            if problem or not self.still:
                return 409, dict(error=problem or '请等待底盘停稳')
            path, revision = self.paths.pop(key)
            if self.n.maps.meta['revision'] != revision:
                return 409, dict(error='地图已更新，请重新预览')
            p = path.poses[0].pose.position
            robot = self.n.robot_map
            if math.hypot(robot['x']-p.x, robot['y']-p.y) > .15:
                return 409, dict(error='机器人起点已变化，请重新预览')
            self.last_id = self.goal = key
            self.phase, self.message = 'checking', '正在复核路径'
            self.phase_since = time.monotonic()
            try:
                future = self.check.call_async(IsPathValid.Request(path=path))
                future.add_done_callback(lambda f: self.checked(f, key, path))
            except Exception:
                self.finish('error', '路径校验请求失败')
                raise
            return 202, dict(id=key)

    def checked(self, future, key, path):
        with self.lock:
            if self.goal != key or self.phase != 'checking':
                return
            try:
                problem = self.problem()
                if not future.result().is_valid or problem:
                    self.finish('error', problem or '当前代价地图中路径已不可通行')
                    return
                self.pub = self.n.create_publisher(TwistStamped, HARDWARE, qos_profile_sensor_data)
                self.speed_profile = PathSpeedProfile([(p.pose.position.x,p.pose.position.y) for p in path.poses])
                self.update_speed_limit()
                self.phase, self.message = 'starting', '正在启动路径跟踪'
                self.phase_since = time.monotonic()
                goal = FollowPath.Goal(path=path, controller_id='FollowPath', goal_checker_id='general_goal_checker')
                self.client.send_goal_async(goal).add_done_callback(lambda f: self.accepted(f, key))
            except Exception as exc:
                self.finish('error', '路径跟踪启动失败')
                self.n.get_logger().error(str(exc))

    def accepted(self, future, key):
        with self.lock:
            try:
                h = future.result()
                if self.goal != key or self.phase != 'starting':
                    if h.accepted:
                        h.cancel_goal_async()
                    return
                if not h.accepted:
                    self.finish('error', '控制器拒绝路径')
                    return
                self.handle = h
                self.phase, self.message = 'running', '机器人正在沿路径移动'
                self.accepted_at = time.monotonic()
                h.get_result_async().add_done_callback(lambda f: self.completed(f,key))
            except Exception:
                self.finish('error', '跟踪动作连接失败')

    def completed(self, future, key):
        with self.lock:
            if self.goal != key:
                return
            try:
                success = future.result().status == GoalStatus.STATUS_SUCCEEDED
            except Exception:
                success = False
            self.finish('arrived' if success else 'stopped', '已到达目标' if success else '路径跟踪已停止')

    def send(self, cmd=None):
        if self.pub is None:
            return
        out = TwistStamped()
        out.header.frame_id = 'base_link'
        out.header.stamp = self.n.get_clock().now().to_msg()
        if cmd is not None:
            x,y,w = cmd.linear.x,cmd.linear.y,cmd.angular.z
            if not all(math.isfinite(v) for v in (x,y,w)):
                raise ValueError('非有限速度')
            scale = self.speed / max(self.speed, math.hypot(x,y))
            out.twist.linear.x, out.twist.linear.y = x*scale, y*scale
            out.twist.angular.z = max(-self.yaw_speed,min(self.yaw_speed,w))
        self.pub.publish(out)

    def finish(self, phase, message):
        handle = self.handle
        self.handle = self.goal = None
        self.speed_profile = None
        self.speed_limit_pub.publish(SpeedLimit(percentage=False,speed_limit=0.))
        self.phase, self.message = phase, message
        self.send()
        self.stop_until = time.monotonic()+.6
        if handle and phase not in ('arrived','stopped'):
            handle.cancel_goal_async()

    def cancel(self, key=None):
        with self.lock:
            if key:
                self.paths.pop(key,None)
            if self.goal and (key is None or self.goal == key):
                self.finish('cancelled', '导航已取消，已下发停车指令')
            return 200, dict(message=self.message)

    def tick(self):
        with self.lock:
            now = time.monotonic()
            if self.goal:
                self.update_speed_limit()
                issue = self.problem()
                if issue:
                    self.finish('error', issue)
                elif self.phase != 'running':
                    self.send()
                    if now-self.phase_since > 5:
                        self.finish('error','控制器响应超时')
                elif self.safe_at <= self.accepted_at or now-self.safe_at > .2:
                    self.send()
                    if now-max(self.accepted_at,self.safe_at) > 1:
                        self.finish('error','避障速度反馈中断')
                else:
                    try:
                        self.send(self.command)
                    except ValueError:
                        self.finish('error','控制器速度无效')
            elif self.pub:
                self.send()
                if now >= self.stop_until:
                    self.n.destroy_publisher(self.pub)
                    self.pub = None

    def update_speed_limit(self):
        if self.speed_profile is None or self.n.robot_map is None:
            return
        pose = self.n.robot_map
        limit = self.speed_profile.limit(pose['x'],pose['y'],self.speed)
        # Humble restores its original limits only on the zero sentinel.
        msg = SpeedLimit(percentage=False,speed_limit=0. if limit>=self.speed-1e-6 else limit)
        msg.header.stamp = self.n.get_clock().now().to_msg()
        self.speed_limit_pub.publish(msg)
