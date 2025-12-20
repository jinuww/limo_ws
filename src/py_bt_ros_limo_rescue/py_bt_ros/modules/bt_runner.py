import os
import pygame
from geometry_msgs.msg import PoseStamped
from modules.agent import Agent

class BTRunner:
    def __init__(self, config):
        self.config = config
        self.bt_viz_cfg = config['bt_runner'].get('bt_visualiser', {})
        self.bt_tick_rate = config['bt_runner']['bt_tick_rate']
        pygame.init()
        if self.bt_viz_cfg.get('enabled', False):
            os.environ['SDL_VIDEO_WINDOW_POS'] = "0,30"  # top-left corner
            self.screen_height = self.bt_viz_cfg.get('screen_height', 500)
            self.screen_width = self.bt_viz_cfg.get('screen_width', 500)
            self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.RESIZABLE)
            self.background_color = (224, 224, 224)
            from .bt_visualiser import BTViewer
            self.bt_visualiser = BTViewer(
                direction=self.bt_viz_cfg.get('direction', 'Vertical')
            )
        self.clock = pygame.time.Clock()

        self.reset()

    def _make_pose(self, x, y, yaw=0.0, frame_id="map"):
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        return ps


    def reset(self):
        self.running = True
        self.paused = False
        self.agent = None

        ros_namespace = self.config['agent'].get('namespaces', [])
        self.agent = Agent(ros_namespace)

        scenario_path = self.config['scenario'].replace('.', '/')
        behavior_tree_xml = f"{os.path.dirname(os.path.dirname(os.path.abspath(__file__)))}/{scenario_path}/{self.config['agent']['behavior_tree_xml']}"
        self.agent.create_behavior_tree(str(behavior_tree_xml))


        bb = self.agent.blackboard
        bb_cfg = self.config.get('blackboard', {})

        if 'patrol_points' in bb_cfg:
            parsed_points = []
            for p in bb_cfg['patrol_points']:
                ps = PoseStamped()
                ps.header.frame_id = p['header']['frame_id']
                
                ps.pose.position.x = float(p['pose']['position']['x'])
                ps.pose.position.y = float(p['pose']['position']['y'])
                ps.pose.position.z = float(p['pose']['position']['z'])

                ori = p['pose']['orientation']
                ps.pose.orientation.x = float(ori.get('x', 0.0))
                ps.pose.orientation.y = float(ori.get('y', 0.0))
                ps.pose.orientation.z = float(ori.get('z', 0.0)) 
                ps.pose.orientation.w = float(ori.get('w', 1.0))
                
                parsed_points.append(ps)
            bb["patrol_points"] = parsed_points
            print(f"Config Loaded: {len(parsed_points)} Patrol Points")

        if 'exit_pose' in bb_cfg:
            p = bb_cfg['exit_pose']
            ps = PoseStamped()
            ps.header.frame_id = p['header']['frame_id']
            ps.pose.position.x = float(p['pose']['position']['x'])
            ps.pose.position.y = float(p['pose']['position']['y'])
            ps.pose.position.z = float(p['pose']['position']['z'])
            
            ori = p['pose']['orientation']
            ps.pose.orientation.x = float(ori.get('x', 0.0))
            ps.pose.orientation.y = float(ori.get('y', 0.0))
            ps.pose.orientation.z = float(ori.get('z', 0.0)) 
            ps.pose.orientation.w = float(ori.get('w', 1.0))
            
            bb["exit_pose"] = ps
            print("Config Loaded: Exit Pose")
        
        bb["image_captured"] = False

    async def step(self):
        await self.agent.run_tree()
        self.clock.tick(self.bt_tick_rate)

    def close(self):
        pass

    def render(self):
        if self.bt_viz_cfg.get('enabled', False):
            self.bt_visualiser.render_tree(self.screen, self.agent.tree)

            if self.paused:
                font = pygame.font.Font(None, 48)
                text = font.render("Paused", True, (255, 0, 0))
                self.screen.blit(text, (10, 10))

            pygame.display.flip()

    def handle_keyboard_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    self.running = False
                elif event.key == pygame.K_p:
                    self.paused = not self.paused

