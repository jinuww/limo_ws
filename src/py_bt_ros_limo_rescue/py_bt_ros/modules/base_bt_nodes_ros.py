from modules.base_bt_nodes import Node, Status
from rclpy.action import ActionClient


class ConditionWithROSTopics(Node):
    def __init__(self, name, agent, msg_types_topics):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self._cache = {}
        for msg_type, topic, key in msg_types_topics:
            self.ros.node.create_subscription(
                msg_type, topic,
                lambda m, k=key: self._cache.__setitem__(k, m),
                1
            )
        # For PA-BT
        self.is_expanded = False
        self.type = "Condition"

    #async def run(self, agent, blackboard):
    #    if not self._cache:
    #        self.status = Status.RUNNING
    #    elif self._predicate(agent, blackboard):
    #        self.status = Status.SUCCESS
    #    else:
    #        self.status = Status.FAILURE
    #    
    #    # For PA-BT
    #    blackboard[self.name] = {'status': self.status, 'is_expanded': self.is_expanded} 
#
 #       return self.status
async def run(self, agent, blackboard):
    if not self._cache:
        self.status = Status.FAILURE
    elif self._predicate(agent, blackboard):
        self.status = Status.SUCCESS
    else:
        self.status = Status.FAILURE

    blackboard[self.name] = {'status': self.status, 'is_expanded': self.is_expanded}
    return self.status


    def _predicate(self, agent, blackboard) -> bool: 
        raise NotImplementedError

    def set_expanded(self): # For PA-BT
        self.is_expanded = True


class ActionWithROSAction(Node):
    def __init__(self, name, agent, action_spec):
        super().__init__(name)
        self.ros = agent.ros_bridge
        action_type, action_name = action_spec
        self.client = ActionClient(self.ros.node, action_type, action_name)

        self._goal_handle = None
        self._result_future = None
        self._phase = 'idle'       # 'idle' -> 'sending' -> 'running'

        # For PA-BT
        self.type = "Action"

    def _build_goal(self, agent, blackboard):
        raise NotImplementedError   

    def _on_running(self, agent, blackboard):
        pass

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        return Status.SUCCESS                    

    async def run(self, agent, blackboard):
        # Action Request 송신
        if self._phase == 'idle':
            if not self.client.wait_for_server(timeout_sec=0.0): # 서버가 아직 준비가 안된 상황 고려
                self.status = Status.RUNNING
                return self.status

            goal = self._build_goal(agent, blackboard)
            if goal is None:
                self.status = Status.FAILURE
                return self.status

            self.client.send_goal_async(goal).add_done_callback(self._on_goal_response)
            self._phase = 'sending'
            self.status = Status.RUNNING
            return self.status

        if self._phase == 'sending':
            self.status = Status.RUNNING
            return self.status

        if self._phase == 'running':
            if self._result_future and self._result_future.done():
                res = self._result_future.result()   # <- get_result 응답
                self.status = self._interpret_result(res.result, agent, blackboard, res.status)
                self._phase = 'idle'
                return self.status
            self._on_running(agent, blackboard)
            self.status = Status.RUNNING
            return self.status

        self.status = Status.RUNNING
        return self.status

    def _on_goal_response(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self._phase = 'idle'
            return
        self._result_future = self._goal_handle.get_result_async()
        self._phase = 'running'

    def halt(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._phase = 'idle'


class ActionWithROSService(Node):
    def __init__(self, name, agent, service_spec):
        super().__init__(name)
        self.ros = agent.ros_bridge
        srv_type, srv_name = service_spec
        self.client = self.ros.node.create_client(srv_type, srv_name)

        self._future = None
        self._sent = False

        # For PA-BT
        self.type = "Action"

    def _build_request(self, agent, blackboard):
        raise NotImplementedError

    def _interpret_response(self, response, agent, blackboard):
        return Status.SUCCESS

    async def run(self, agent, blackboard):
        if not self.client.wait_for_service(timeout_sec=0.0):
            self.status = Status.RUNNING
            return self.status

        # 최초 1회 호출
        if not self._sent:
            req = self._build_request(agent, blackboard)
            if req is None:
                self.status = Status.FAILURE
                return self.status
            self._future = self.client.call_async(req)
            self._sent = True
            self.status = Status.RUNNING
            return self.status

        # 응답 도착 확인
        if self._future and self._future.done():
            resp = self._future.result()
            self.status = self._interpret_response(resp, agent, blackboard)
            self._sent = False
            return self.status

        self.status = Status.RUNNING
        return self.status

    def halt(self):
        self._sent = False
