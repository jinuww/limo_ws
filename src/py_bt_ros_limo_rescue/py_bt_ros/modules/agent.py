from modules.utils import config, optional_import
env_pkg = config.get('scenario')
bt_module = optional_import(env_pkg + ".bt_nodes")

from modules.bt_constructor import build_behavior_tree
from modules.ros_bridge import ROSBridge

class Agent:
    def __init__(self, ros_namespace=None):
        self.blackboard = {}
        self.ros_bridge = ROSBridge.get()
        self.ros_namespace = ros_namespace

    def create_behavior_tree(self, behavior_tree_xml):
        self.behavior_tree_xml = behavior_tree_xml
        self.tree = build_behavior_tree(self, behavior_tree_xml, env_pkg)

    # ✅ 필요할 때만 외부에서 호출
    def reset_tree(self):
        if hasattr(self, "tree") and self.tree is not None:
            self.tree.reset()

    async def run_tree(self):
        # 매 tick reset 하면 Nav2 Action이 정상 진행 불가
        # self.reset_tree()
        return await self.tree.run(self, self.blackboard)

