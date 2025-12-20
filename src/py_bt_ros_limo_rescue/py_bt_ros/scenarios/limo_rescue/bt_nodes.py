import math
import numpy as np
import asyncio
if not hasattr(np, 'float'):
    np.float = float  

from modules.base_bt_nodes import (
    BTNodeList as _BaseBTNodeList,
    Status as Status,
    Sequence as _Sequence,
    Fallback as _Fallback,
    ReactiveSequence as _ReactiveSequence,
    ReactiveFallback as _ReactiveFallback,
    Parallel as _Parallel,
)

from modules.base_bt_nodes_ros import (
    ConditionWithROSTopics,
    ActionWithROSAction,
    ActionWithROSService,
)

from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from rclpy.time import Time


class BTNodeList(_BaseBTNodeList):
    ACTION_NODES = _BaseBTNodeList.ACTION_NODES + [
        "SetNextPoint", "MoveToPoint", "ApproachToTarget", 
        "MakeRescueVoice", "NavigateToExit", "Wait" 
    ]
    CONDITION_NODES = _BaseBTNodeList.CONDITION_NODES + [
        "IsTargetDetected", "IsTargetClose", "AtExit",
    ]

class IsTargetDetected(ConditionWithROSTopics):
    def __init__(self, xml_tag, agent, name=None):
        actual_name = name if name else xml_tag
        super().__init__(actual_name, agent, [
            (Point, "/detected_person_target", "yolo_msg")
        ])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, agent.ros_bridge.node)

    def _predicate(self, agent, blackboard) -> bool:
        if "yolo_msg" in self._cache:
            msg = self._cache["yolo_msg"]


            try:
                # 좌표 변환 시도
                trans = self.tf_buffer.lookup_transform('map', 'base_link', Time())
                
                # 변환 로직
                robot_x = trans.transform.translation.x
                robot_y = trans.transform.translation.y
                q = trans.transform.rotation
                (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

                rel_angle = msg.x
                rel_dist = msg.y
                target_angle_rad = yaw + math.radians(rel_angle)
                
                map_x = robot_x + (rel_dist * math.cos(target_angle_rad))
                map_y = robot_y + (rel_dist * math.sin(target_angle_rad))

                p = PoseStamped()
                p.header.frame_id = "map"
                p.pose.position.x = map_x
                p.pose.position.y = map_y
                p.pose.orientation.w = 1.0
                
                # 블랙보드 저장
                blackboard["target_map_pose"] = p
                
                print(f"타겟 확정 맵 좌표: ({map_x:.2f}, {map_y:.2f})")
                return True

            except Exception as e:
                print(f"좌표 변환 실패 이유: {e}")
                return False

        # 2. 기억(Memory) 확인
        stored_pose = blackboard.get("target_map_pose")
        if stored_pose is not None:
            # print("기억 타겟 유지")
            return True

        return False


class IsTargetClose(ConditionWithROSTopics):
    def __init__(self, xml_tag, agent, name=None, close_dist=0.3):
        actual_name = name if name else xml_tag
        self.stop_distance = float(close_dist)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, agent.ros_bridge.node)
        super().__init__(actual_name, agent, [
            (Point, "/detected_person_target", "yolo_msg")
        ])

    def _predicate(self, agent, blackboard) -> bool:
        if blackboard.get("approach_finished") is True:
            return True

        target_pose = blackboard.get("target_map_pose")
        if target_pose is not None:
            try:
                trans = self.tf_buffer.lookup_transform('map', 'base_link', Time())
                rx = trans.transform.translation.x
                ry = trans.transform.translation.y
                tx = target_pose.pose.position.x
                ty = target_pose.pose.position.y
                current_dist = math.hypot(tx - rx, ty - ry)
                
                if current_dist <= (self.stop_distance + 0.2):
                    blackboard["approach_finished"] = True
                    return True
            except Exception:
                pass
        
        if "yolo_msg" in self._cache:
            dist = self._cache["yolo_msg"].y
            if dist <= (self.stop_distance + 0.2):
                blackboard["approach_finished"] = True
                return True
        return False

# class AtExit(ConditionWithROSTopics):
#     def __init__(self, xml_tag, agent, name=None):
#         actual_name = name if name else xml_tag
#         super().__init__(actual_name, agent, [])
#     def _predicate(self, agent, blackboard) -> bool:
#         return blackboard.get("is_evac_finished", False)
# bt_nodes.py 내부의 AtExit 클래스를 이것으로 덮어씌우세요.

class AtExit(ConditionWithROSTopics):
    def __init__(self, xml_tag, agent, name=None, dist_thres=1.0):
        actual_name = name if name else xml_tag
        super().__init__(actual_name, agent, [])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, agent.ros_bridge.node)
        self.dist_thres = float(dist_thres) # 도착 인정 범위 (미터 단위)

    def _predicate(self, agent, blackboard) -> bool:
        if blackboard.get("is_evac_finished", False) is True:
            return True

        exit_pose = blackboard.get("exit_pose")
        if exit_pose is None:
            return False

        try:
            # 로봇의 현재 위치(base_link)
            trans = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
            
            # 비상구 위치
            ex = exit_pose.pose.position.x
            ey = exit_pose.pose.position.y

            # 거리 계산
            dist = math.hypot(ex - rx, ey - ry)

            if dist < self.dist_thres:
                print(f"비상구 도착 (거리: {dist:.2f}m)")
                blackboard["is_evac_finished"] = True # 플래그 강제 설정
                return True
                
        except Exception:
            pass 
        
        return False


class SetNextPoint(ActionWithROSService):
    def __init__(self, xml_tag, agent, name=None, **kwargs):
        actual_name = name if name else xml_tag
        super().__init__(actual_name, agent, (Trigger, "/bt/dummy_service"))
        self._idx = -1
    def _build_request(self, agent, blackboard):
        return Trigger.Request()
    async def run(self, agent, blackboard):
        pts = blackboard.get("patrol_points")
        if not pts: return Status.FAILURE
        self._idx = (self._idx + 1) % len(pts)
        blackboard["next_point"] = pts[self._idx]
        return Status.SUCCESS

class MoveToPoint(ActionWithROSAction):
    def __init__(self, xml_tag, agent, name=None):
        actual_name = name if name else xml_tag
        super().__init__(actual_name, agent, (NavigateToPose, "/navigate_to_pose"))

        self.voice_pub = agent.ros_bridge.node.create_publisher(String, "/voice_cmd", 10)

    def _build_goal(self, agent, blackboard):
        self.voice_pub.publish(String(data="beep_start"))
        return NavigateToPose.Goal(pose=blackboard.get("next_point"))

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        self.voice_pub.publish(String(data="beep_stop"))
        
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            return Status.SUCCESS
        return Status.FAILURE

class ApproachToTarget(ActionWithROSAction):
    def __init__(self, xml_tag, agent, name=None):
        actual_name = name if name else xml_tag
        super().__init__(actual_name, agent, (NavigateToPose, "/navigate_to_pose"))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, agent.ros_bridge.node)
        # 0.5m 앞 정지
        self.STOP_DISTANCE = 0.5

    def _build_goal(self, agent, blackboard):
        target_pose = blackboard.get("target_map_pose")
        if target_pose is None: return None

        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y
            
            target_x = target_pose.pose.position.x
            target_y = target_pose.pose.position.y
            
            angle_to_target = math.atan2(target_y - robot_y, target_x - robot_x)
            dist_to_target = math.hypot(target_x - robot_x, target_y - robot_y)
            travel_dist = max(0.0, dist_to_target - self.STOP_DISTANCE)
            
            goal_x = robot_x + (travel_dist * math.cos(angle_to_target))
            goal_y = robot_y + (travel_dist * math.sin(angle_to_target))

            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = agent.ros_bridge.node.get_clock().now().to_msg()
            goal.pose.pose.position.x = goal_x
            goal.pose.pose.position.y = goal_y
            goal.pose.pose.orientation.z = math.sin(angle_to_target / 2.0)
            goal.pose.pose.orientation.w = math.cos(angle_to_target / 2.0)
            
            print(f"[Approach] (남은거리: {travel_dist:.2f}m)")
            return goal

        except Exception:
            return None

    def _interpret_result(self, result, agent, blackboard, status_code=None):
            if status_code == GoalStatus.STATUS_SUCCEEDED:
                blackboard["approach_finished"] = True
                
                print("도착 완료!")
                return Status.FAILURE
            
    
    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            blackboard["approach_finished"] = True
            return Status.SUCCESS
        return Status.FAILURE

class Wait(ActionWithROSService):
    def __init__(self, xml_tag, agent, name=None, duration=2.0):
        actual_name = name if name else xml_tag
        super().__init__(actual_name, agent, (Trigger, "/bt/dummy_wait"))
        self.duration = float(duration)

    def _build_request(self, agent, blackboard):
        return Trigger.Request()

    async def run(self, agent, blackboard):
        print(f"⏳ [Wait] {self.duration}초간 대기합니다...")
        await asyncio.sleep(self.duration)
        return Status.SUCCESS


class MakeRescueVoice(ActionWithROSService):
    def __init__(self, xml_tag, agent, name=None):
        actual_name = name if name else xml_tag
        super().__init__(actual_name, agent, (Trigger, "/bt/dummy_voice_srv"))
        self.pub = agent.ros_bridge.node.create_publisher(String, "/voice_cmd", 10)

    def _build_request(self, agent, blackboard):
        return Trigger.Request()

    async def run(self, agent, blackboard):
        msg = String()
        msg.data = "follow_me" 
        self.pub.publish(msg)
        
        self.pub.publish(String(data="beep_stop"))
        
        print("저를 따라오세요!")
        await asyncio.sleep(3.0) 
        return Status.SUCCESS

class NavigateToExit(ActionWithROSAction):
    def __init__(self, xml_tag, agent, name=None):
        actual_name = name if name else xml_tag
        super().__init__(actual_name, agent, (NavigateToPose, "/navigate_to_pose"))
        # Voice 명령 퍼블리셔
        self.voice_pub = agent.ros_bridge.node.create_publisher(String, "/voice_cmd", 10)

    def _build_goal(self, agent, blackboard):
        print("비상구로 안내합니다")
        self.voice_pub.publish(String(data="beep_start"))
        return NavigateToPose.Goal(pose=blackboard.get("exit_pose"))
    
    def _interpret_result(self, result, agent, blackboard, status_code=None):
        self.voice_pub.publish(String(data="beep_stop"))

        if status_code == GoalStatus.STATUS_SUCCEEDED:
            blackboard["is_evac_finished"] = True
            
            print("비상구 도착")
            self.voice_pub.publish(String(data="arrived_exit"))
            
            return Status.SUCCESS
            
        return Status.FAILURE
    
# --- Wrapper Classes ---
class Sequence(_Sequence):
    def __init__(self, *args, children=None, name=None, **kwargs):
        super().__init__(name or "Sequence", children or [])
class Fallback(_Fallback):
    def __init__(self, *args, children=None, name=None, **kwargs):
        super().__init__(name or "Fallback", children or [])
class ReactiveSequence(_ReactiveSequence):
    def __init__(self, *args, children=None, name=None, **kwargs):
        super().__init__(name or "ReactiveSequence", children or [])
class ReactiveFallback(_ReactiveFallback):
    def __init__(self, *args, children=None, name=None, **kwargs):
        super().__init__(name or "ReactiveFallback", children or [])
class Parallel(_Parallel):
    def __init__(self, *args, children=None, name=None, **kwargs):
        super().__init__(name or "Parallel", children or [])
