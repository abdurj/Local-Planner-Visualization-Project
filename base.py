import threading

import pygame, random, pygame_gui
from pygame.locals import *
from planners.prm import ProbabilisticRoadmap
from search.search import Dijkstra

lock = threading.Lock()

class Color:
    WHITE = (255, 255, 255)
    GREY = (70, 70, 70)
    BLUE = (0, 0, 255)
    GREEN = (0, 255, 0)
    RED = (255, 0, 0)


class State:
    PRM = 0
    RRT = 1
    PF = 2


"""
    map_pos -> Tuple(x,y)
    map_dim -> Tuple(w,h)
    obs_dim -> Tuple(maxw,maxh)
"""


def sample_envir(map_pos, map_dim, obs_dim):
    sx = map_pos[0]
    sy = map_pos[1]
    ex = sx + map_dim[0] - obs_dim[0]
    ey = sy + map_dim[1] - obs_dim[1]
    x = int(random.uniform(sx, ex))
    y = int(random.uniform(sy, ey))

    return (x, y)


def localize(map, pos):
    return (pos[0] - map[0], pos[1] - map[1])


# TODO: start, goal collision checking
def generate_obs(num_obstacles, map_pos, map_dim, obs_dim):
    obs = []
    for i in range(num_obstacles):
        rect = None
        collision = True
        while collision:
            pos = sample_envir(map_pos, map_dim, obs_dim)
            size = (int(random.uniform(10, obs_dim[0])), int(random.uniform(10, obs_dim[1])))
            rect = pygame.Rect(pos, size)
            collision = False
            for obj in obs:
                if rect.colliderect(obj):
                    collision = True
                    break
        obs.append(rect)
    return obs


class App:
    def __init__(self):
        self.sc = 0.8
        self._running = True
        self._display_surf = None
        self.size = self.width, self.height = int(1400 * self.sc), int(900 * self.sc)

        self.map = None
        self.map_pos = (3, int(90 * self.sc))
        self.map_size = self.mapw, self.maph = int(1100 * self.sc), int(807 * self.sc)

        self.clock = pygame.time.Clock()
        self.dt = 0

        self.toolbar_pos = (3, 3)
        self.toolbar_size = self.toolbarw, self.toolbarh = int(1393 * self.sc), int(83 * self.sc)

        self.state = State.PRM

        self.optionp_pos = (int(1106 * self.sc), int(90 * self.sc))
        self.optionp_size = self.optionp_w, self.optionp_h = int(290 * self.sc), int(807 * self.sc)

        self.planners = ['Probabilistic Roadmap', "RRT", "Potential Field"]
        self.default_planner = 'Probabilistic Roadmap'

        self.obstacles = []
        self.obs_dim = (50, 50)
        self._rect_start_pos = None
        self.num_obstacles = 20

        self.start_pose = self.sx, self.sy = (20, 20)
        self.start_radius = 10
        self.goal_pose = self.gx, self.gy = (600, 600)
        self.goal_radius = 20
        self.node_radius = 5

        self.transition_pose = None

        self.path = []

        # PRM OPTIONS
        self.prm_options = {
            'sample_size': 500,
            'neighbours': 5
        }

        # RRT OPTIONS
        self.rrt_options = {
            'bias': 10
        }

        # FLAGS
        self.flags = {
            'set_obs': False,
            'drag_start': False,
            'drag_goal': False
        }

    def on_init(self):
        pygame.init()
        self._display_surf = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.manager = pygame_gui.UIManager(self.size, 'theme.json')
        self.map = pygame.Surface.subsurface(self._display_surf, (self.map_pos, self.map_size))
        self.toolbar_base = pygame_gui.core.UIContainer(
            pygame.Rect(self.toolbar_pos, (self.toolbarw, self.toolbarh + 200)), manager=self.manager,
            starting_height=0)
        self.toolbar_ui = pygame_gui.elements.UIPanel(pygame.Rect((0, 0), self.toolbar_size), starting_layer_height=1,
                                                      container=self.toolbar_base, manager=self.manager,
                                                      object_id='toolbar')
        self.toolbar_buttons = {
            'planner_select': pygame_gui.elements.UIDropDownMenu(self.planners, self.default_planner,
                                                                 pygame.Rect((42, 15), (243, 58)), manager=self.manager,
                                                                 container=self.toolbar_base,
                                                                 expansion_height_limit=100),
            'add_obs': pygame_gui.elements.UIButton(pygame.Rect((399, 12), (243, 58)), 'Add Obstacles',
                                                    manager=self.manager, container=self.toolbar_ui),
            'generate_obs': pygame_gui.elements.UIButton(pygame.Rect((756, 12), (243, 58)), 'Generate Obstacles',
                                                         manager=self.manager, container=self.toolbar_ui),
            'reset_obs': pygame_gui.elements.UIButton(pygame.Rect((1113, 12), (243, 58)), 'Reset Obstacles',
                                                      manager=self.manager, container=self.toolbar_ui)
        }

        self.option_ui_panel = pygame_gui.elements.UIPanel(
            relative_rect=pygame.Rect(self.optionp_pos, self.optionp_size), manager=self.manager,
            element_id='option_panel', starting_layer_height=0)

        self.option_ui_windows = {
            State.RRT: pygame_gui.core.UIContainer(
                relative_rect=pygame.Rect((0, self.optionp_h / 3.5), self.optionp_size), manager=self.manager,
                object_id='rrt', container=self.option_ui_panel),
            State.PRM: pygame_gui.core.UIContainer(
                relative_rect=pygame.Rect((0, self.optionp_h / 3.5), self.optionp_size), manager=self.manager,
                object_id='prm', container=self.option_ui_panel),
            State.PF: pygame_gui.core.UIContainer(
                relative_rect=pygame.Rect((0, self.optionp_h / 3.5), self.optionp_size), manager=self.manager,
                object_id='pf', container=self.option_ui_panel),
        }

        self.option_title = pygame_gui.elements.UITextBox('General Options', pygame.Rect(11, 17, 268, 61),
                                                          manager=self.manager, container=self.option_ui_panel)
        self.obstacle_textbox = pygame_gui.elements.UITextBox(f'Generate {self.num_obstacles} Obstacles',
                                                              pygame.Rect(11, 110, 268, 40), manager=self.manager,
                                                              container=self.option_ui_panel)
        self.obstacle_slider = pygame_gui.elements.UIHorizontalSlider(pygame.Rect(10, 150, 270, 40), self.num_obstacles,
                                                                      (1, 50), manager=self.manager,
                                                                      container=self.option_ui_panel)
        self.visualize_button = pygame_gui.elements.UIButton(pygame.Rect(18, 729, 255, 60), "Simulate!",
                                                             manager=self.manager, container=self.option_ui_panel)

        self.rrt_ui_options = {
            'title': pygame_gui.elements.UITextBox('RRT Options', pygame.Rect(11, 17, 268, 61), manager=self.manager,
                                                   container=self.option_ui_windows[State.RRT]),
            'bias_slider_textbox': pygame_gui.elements.UITextBox(f'Bias: {self.rrt_options["bias"]}',
                                                                 pygame.Rect(11, 110, 268, 40), manager=self.manager,
                                                                 container=self.option_ui_windows[State.RRT]),
            'bias_slider': pygame_gui.elements.UIHorizontalSlider(pygame.Rect(10, 150, 270, 40), 0.1, (0, 1),
                                                                  manager=self.manager,
                                                                  container=self.option_ui_windows[State.RRT])
        }

        self.prm_ui_options = {
            'title': pygame_gui.elements.UITextBox('PRM Options', pygame.Rect(11, 17, 268, 61), manager=self.manager,
                                                   container=self.option_ui_windows[State.PRM]),
            'sample_slider_textbox': pygame_gui.elements.UITextBox(f'Sample Size: {self.prm_options["sample_size"]}',
                                                                   pygame.Rect(11, 110, 268, 40), manager=self.manager,
                                                                   container=self.option_ui_windows[State.PRM]),
            'sample_slider': pygame_gui.elements.UIHorizontalSlider(pygame.Rect(10, 150, 270, 40),
                                                                    self.prm_options['sample_size'], (100, 1500),
                                                                    manager=self.manager,
                                                                    container=self.option_ui_windows[State.PRM]),
            'neighbour_slider_textbox': pygame_gui.elements.UITextBox(
                f'Connect to {self.prm_options["neighbours"]} neighbours', pygame.Rect(11, 200, 268, 40),
                manager=self.manager, container=self.option_ui_windows[State.PRM]),
            'neighbour_slider': pygame_gui.elements.UIHorizontalSlider(pygame.Rect(10, 240, 270, 40),
                                                                       self.prm_options['neighbours'], (1, 10),
                                                                       manager=self.manager,
                                                                       container=self.option_ui_windows[State.PRM]),
            'set_k': pygame_gui.elements.UIButton(pygame.Rect(10,280,270,40), 'Set Neighbours', manager=self.manager, container=self.option_ui_windows[State.PRM])
        }

        self.change_state(self.default_planner)
        self._running = True

        self.planner = ProbabilisticRoadmap(self.map_size, self.start_pose, self.start_radius, self.goal_pose, self.goal_radius, self.obstacles,
                                            self.prm_options['neighbours'])

        self.t = threading.Thread(target=self.planner.sample, args=(self.prm_options['sample_size'], ))
        self.t.start()

        self.search = Dijkstra(self.planner.nodes)

    def on_event(self, event):
        if event.type == pygame.QUIT:
            self._running = False
        if event.type == pygame.MOUSEBUTTONDOWN:
            if (self.flags['set_obs']):
                self._rect_start_pos = localize(self.map_pos, pygame.mouse.get_pos())
                return

            pos = localize(self.map_pos, pygame.mouse.get_pos())
            if ((pos[0] - self.sx) ** 2 + (pos[1] - self.sy) ** 2 < self.start_radius ** 2):
                self.flags['drag_start'] = True
                self.transition_pose = self.start_pose

            if ((pos[0] - self.gx) ** 2 + (pos[1] - self.gy) ** 2 < self.goal_radius ** 2):
                self.flags['drag_goal'] = True
                self.transition_pose = self.goal_pose

        if event.type == pygame.MOUSEBUTTONUP:
            if (self.flags['set_obs'] and self._rect_start_pos):
                curr_pos = localize(self.map_pos, pygame.mouse.get_pos())
                w = curr_pos[0] - self._rect_start_pos[0]
                h = curr_pos[1] - self._rect_start_pos[1]
                rect = pygame.Rect(*self._rect_start_pos, w, h)
                rect.normalize()
                self.obstacles.append(rect)
                self._rect_start_pos = None
                self.update_obstacles()

            elif (self.flags['drag_start']):
                self.flags['drag_start'] = False
                collision = False
                for obs in self.obstacles:
                    if obs.collidepoint(self.transition_pose):
                        collision = True
                        break
                if not collision:
                    self.start_pose = self.transition_pose
                    self.planner.update_pose(self.start_pose, self.goal_pose)
                self.transition_pose = None
            elif (self.flags['drag_goal']):
                self.flags['drag_goal'] = False
                collision = False
                for obs in self.obstacles:
                    if obs.collidepoint(self.transition_pose):
                        collision = True
                        break
                if not collision:
                    self.goal_pose = self.transition_pose
                    self.planner.update_pose(self.start_pose, self.goal_pose)
                self.transition_pose = None

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                if (self.flags['set_obs']):
                    self.flags['set_obs'] = False
                    self._rect_start_pos = None
                if (self.flags['drag_start'] or self.flags['drag_goal']):
                    self.transition_pose = None
                    self.flags['drag_start'] = False
                    self.flags['drag_goal'] = False

    def on_ui_event(self, event):
        if event.type == pygame.USEREVENT:
            if event.user_type == pygame_gui.UI_DROP_DOWN_MENU_CHANGED:
                if event.ui_element == self.toolbar_buttons['planner_select']:
                    self.change_state(event.text)
            if event.user_type == pygame_gui.UI_HORIZONTAL_SLIDER_MOVED:
                if event.ui_element == self.obstacle_slider:
                    self.num_obstacles = event.value
                    self.obstacle_textbox.kill()
                    self.obstacle_textbox = pygame_gui.elements.UITextBox(f'Generate {self.num_obstacles} Obstacles',
                                                                          pygame.Rect(11, 110, 268, 40),
                                                                          manager=self.manager,
                                                                          container=self.option_ui_panel)

                if event.ui_element == self.rrt_ui_options['bias_slider']:
                    self.rrt_options['bias'] = round(event.value, 2)
                    self.rrt_ui_options['bias_slider_textbox'].kill()
                    self.rrt_ui_options['bias_slider_textbox'] = pygame_gui.elements.UITextBox(
                        f'Bias: {self.rrt_options["bias"]}', pygame.Rect(11, 110, 268, 40), manager=self.manager,
                        container=self.option_ui_windows[State.RRT])

                if event.ui_element == self.prm_ui_options['sample_slider']:
                    self.prm_options['sample_size'] = event.value
                    self.prm_ui_options['sample_slider_textbox'].kill()
                    self.prm_ui_options['sample_slider_textbox'] = pygame_gui.elements.UITextBox(
                        f'Sample Size: {self.prm_options["sample_size"]}', pygame.Rect(11, 110, 268, 40),
                        manager=self.manager,
                        container=self.option_ui_windows[State.PRM])
                    self.update_prm_samples()

                if event.ui_element == self.prm_ui_options['neighbour_slider']:
                    self.prm_options['neighbours'] = event.value
                    self.prm_ui_options['neighbour_slider_textbox'].kill()
                    self.prm_ui_options['neighbour_slider_textbox'] = pygame_gui.elements.UITextBox(
                        f'Connect to {self.prm_options["neighbours"]} neighbours', pygame.Rect(11, 200, 268, 40),
                        manager=self.manager,
                        container=self.option_ui_windows[State.PRM])

            if event.user_type == pygame_gui.UI_BUTTON_PRESSED:
                if event.ui_element == self.toolbar_buttons['add_obs']:
                    self.flags['set_obs'] = True
                if event.ui_element == self.toolbar_buttons['generate_obs']:
                    self.obstacles = generate_obs(self.num_obstacles, self.map_pos, self.map_size, self.obs_dim)
                    self.update_obstacles()
                if event.ui_element == self.toolbar_buttons['reset_obs']:
                    self.obstacles = []
                if event.ui_element == self.visualize_button:
                    self.planner.create_network(self.map, self.node_radius)
                    self.t = threading.Thread(target=self.search.solve, args=(self.planner.nodes, self.planner.get_start_node(), self.planner.get_end_node(),))
                    self.t.start()
                if event.ui_element == self.prm_ui_options['set_k']:
                    print(self.prm_options['neighbours'])
                    self.planner.update_k(self.prm_options['neighbours'])

    def on_loop(self):
        self.dt = self.clock.tick(60) / 1000
        self.manager.update(self.dt)
        if (self.flags['drag_start'] or self.flags['drag_goal']):
            pos = pygame.mouse.get_pos()
            self.transition_pose = (pos[0] - self.map_pos[0], pos[1] - self.map_pos[1])

        self.sx, self.sy = self.start_pose
        self.gx, self.gy = self.goal_pose

    def on_render(self):
        self._display_surf.fill(Color.GREY)
        self.map.fill(Color.WHITE)

        pygame.draw.circle(self.map, Color.GREEN, self.start_pose, self.start_radius)
        pygame.draw.circle(self.map, Color.GREEN, self.goal_pose, self.goal_radius)

        if (self.transition_pose):
            pygame.draw.circle(self.map, Color.RED, self.transition_pose, 15)

        for obj in self.obstacles:
            pygame.draw.rect(self.map, Color.BLUE, obj)

        if self._rect_start_pos:
            curr_pos = localize(self.map_pos, pygame.mouse.get_pos())
            w = curr_pos[0] - self._rect_start_pos[0]
            h = curr_pos[1] - self._rect_start_pos[1]
            rect = pygame.Rect(*self._rect_start_pos, w, h)
            rect.normalize()
            pygame.draw.rect(self.map, Color.BLUE, rect)

        for node in self.planner.nodes:
            node.draw(self.map, self.node_radius, 1)

        for node in self.search.path:
            pygame.draw.circle(self.map, Color.GREEN, node.get_coords(), self.node_radius+2, width=0)

        self.manager.draw_ui(self._display_surf)

        pygame.display.update()

    def on_cleanup(self):
        pygame.quit()

    def on_execute(self):
        if self.on_init() == False:
            self._running = False

        while (self._running):
            for event in pygame.event.get():
                self.on_event(event)
                self.on_ui_event(event)
                self.manager.process_events(event)
            self.on_loop()
            self.on_render()
        self.on_cleanup()

    def change_state(self, state):
        if state == 'Probabilistic Roadmap':
            self.state = State.PRM
        elif state == 'RRT':
            self.state = State.RRT
        elif state == 'Potential Field':
            self.state = State.PF
        self.change_active_options(self.state)

    def change_active_options(self, state):
        for optionp in self.option_ui_windows:
            if optionp == state:
                self.option_ui_windows[optionp].show()
            else:
                self.option_ui_windows[optionp].hide()
        if state == State.PF:
            self.toolbar_buttons['add_obs'].disable()
        else:
            self.toolbar_buttons['add_obs'].enable()

    def update_prm_samples(self):
        self.planner.set_obstacles(self.obstacles)
        self.t = threading.Thread(target=self.planner.sample, args=(self.prm_options['sample_size'],))
        self.t.start()

    def update_obstacles(self):
        self.planner.set_obstacles(self.obstacles)
        self.t = threading.Thread(target=self.planner.sample, args=(self.prm_options['sample_size'],))
        self.t.start()


if __name__ == "__main__":
    theApp = App()
    theApp.on_execute()
