import sys
import threading

import pygame, random, pygame_gui
from pygame.locals import *
from planners.planners import ProbabilisticRoadmap, Color, RRT, PotentialField, CircularObstacle
from search.search import Dijkstra, AStar, GreedyBFS


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
    return pos[0] - map[0], pos[1] - map[1]


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


def generate_circle_obs(num_obstacles, map_pos, map_size, circle_obs_dim, goal_pose):
    obs = []
    for i in range(num_obstacles):
        collision = True
        while collision:
            pos = sample_envir(map_pos, map_size, (circle_obs_dim,circle_obs_dim))
            rad = int(random.uniform(10, circle_obs_dim))
            circle = CircularObstacle(*pos,rad)
            collision = circle.collidepoint(goal_pose)

        obs.append(circle)
    return obs



class App:
    def __init__(self):
        self.sc = 0.8
        self.show_sim = True
        # self.sc = 1
        self._running = True
        self._display_surf = None
        self.size = self.width, self.height = int(1600 * self.sc), int(820 * self.sc) # og: 1600 x 820, test: 2000 x 900

        self.map = None
        self.map_pos = (0, int(0.12 * self.height))
        self.map_size = self.mapw, self.maph = int(0.771 * self.width), int(0.87805 * self.height)

        self.clock = pygame.time.Clock()
        self.dt = 0

        self.toolbar_pos = (-1, 0)
        self.toolbar_size = self.toolbarw, self.toolbarh = int(1.01 * self.width), int(0.122 * self.height)

        self.state = State.PRM

        self.optionp_pos = (int(0.76875 * self.width), int(0.95 * self.toolbarh))
        self.optionp_size = self.optionp_w, self.optionp_h = int(0.234375 * self.width), self.height

        self.planners = ['Probabilistic Roadmap', "RRT", "Potential Field"]
        self.default_planner = 'RRT'
        self.state = State.RRT

        self.searches = ['Dijkstra', 'A*', 'Greedy Best First']
        self.default_search = 'A*'
        self.search = None

        self.obstacles = []
        self.obs_dim = (50, 50)
        self.circle_obs_dim = 50
        self._rect_start_pos = None
        self.num_obstacles = 20

        self.start_pose = self.sx, self.sy = (20, 20)
        self.start_radius = 10
        self.goal_pose = self.gx, self.gy = (int(0.5 * self.mapw), int(0.5 * self.maph))
        self.goal_radius = 20
        self.node_radius = 5

        self.transition_pose = None

        self.path = []

        # PRM OPTIONS
        self.prm_options = {
            'sample_size': 800,
            'neighbours': 10
        }

        # RRT OPTIONS
        self.rrt_options = {
            'bias': 0.1
        }


        self.pf_options = {
            'virtual': True
        }

        # FLAGS
        self.flags = {
            'set_obs': False,
            'drag_start': False,
            'drag_goal': False
        }

        # ON INIT
        self.manager = None
        self.toolbar_base = None
        self.toolbar_ui = None
        self.toolbar_buttons = None
        self.option_ui_panel = None
        self.option_ui_windows = None
        self.option_title = None
        self.obstacle_textbox = None
        self.obstacle_slider = None
        self.visualize_button = None
        self.rrt_ui_options = None
        self.prm_ui_options = None
        self.pf_ui_options = None
        self.planner = None
        self.search = None
        self.t = None

    def on_init(self):
        pygame.init()
        pygame.display.set_caption("Local Planner Visualization Project")
        icon = pygame.image.load(r'images/lpvp.png')
        pygame.display.set_icon(icon)
        self._display_surf = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.manager = pygame_gui.UIManager(self.size, 'theme.json')
        self.map = pygame.Surface.subsurface(self._display_surf, (self.map_pos, self.map_size))
        self.toolbar_base = pygame_gui.core.UIContainer(
            pygame.Rect(self.toolbar_pos, (self.toolbarw, self.toolbarh + 200)), manager=self.manager,
            starting_height=0)
        self.toolbar_ui = pygame_gui.elements.UIPanel(pygame.Rect((0, 0), self.toolbar_size), starting_height=1,
                                                      container=self.toolbar_base, manager=self.manager,
                                                      object_id='toolbar')
        self.toolbar_buttons = {
            'planner_select': pygame_gui.elements.UIDropDownMenu(self.planners, self.default_planner,
                                                                 pygame.Rect((20, 15), (250, 50)), manager=self.manager,
                                                                 container=self.toolbar_base,
                                                                 expansion_height_limit=100),
            'add_obs': pygame_gui.elements.UIButton(relative_rect=pygame.Rect((350, 12), (250, 50)), 
                                                    text='Add Obstacles',
                                                    manager=self.manager, 
                                                    container=self.toolbar_ui,
                                                    anchors={'left': 'left',
                                                                  'right': 'right',
                                                                  'top': 'top',
                                                                  'bottom': 'bottom'}),
            'generate_obs': pygame_gui.elements.UIButton(relative_rect=pygame.Rect((680, 12), (250, 50)), 
                                                         text='Generate Obstacles',
                                                         manager=self.manager, 
                                                         container=self.toolbar_ui,
                                                         anchors={'left': 'left',
                                                                  'right': 'right',
                                                                  'top': 'top',
                                                                  'bottom': 'bottom'}),
            'reset_obs': pygame_gui.elements.UIButton(relative_rect=pygame.Rect((1010, 12), (250, 50)), 
                                                      text='Reset Obstacles',
                                                      manager=self.manager, 
                                                      container=self.toolbar_ui,
                                                      anchors={'left': 'left',
                                                                  'right': 'right',
                                                                  'top': 'top',
                                                                  'bottom': 'bottom'})
        }

        self.option_ui_panel = pygame_gui.elements.UIPanel(
            relative_rect=pygame.Rect(self.optionp_pos, self.optionp_size), manager=self.manager,
            element_id='option_panel', starting_height=0)

        self.option_ui_windows = {
            State.RRT: pygame_gui.core.UIContainer(
                relative_rect=pygame.Rect((0, 120), self.optionp_size), manager=self.manager,
                object_id='rrt', container=self.option_ui_panel),
            State.PRM: pygame_gui.core.UIContainer(
                relative_rect=pygame.Rect((0, 120), self.optionp_size), manager=self.manager,
                object_id='prm', container=self.option_ui_panel),
            State.PF: pygame_gui.core.UIContainer(
                relative_rect=pygame.Rect((0, self.optionp_h / 3.5), self.optionp_size), manager=self.manager,
                object_id='pf', container=self.option_ui_panel),
        }

        self.option_title = pygame_gui.elements.UILabel(relative_rect=pygame.Rect(20, 20, 146, 40),
                                                        text='General Options',
                                                        manager=self.manager, 
                                                        container=self.option_ui_panel,
                                                        object_id='header')
        self.obstacle_textbox = pygame_gui.elements.UILabel(relative_rect=pygame.Rect(20, 60, 180, 35),
                                                            text=f'Generate {self.num_obstacles} Obstacles', 
                                                            manager=self.manager,
                                                            container=self.option_ui_panel)
        self.obstacle_slider = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect(25, 95, 250, 40), 
                                                                      start_value=self.num_obstacles,
                                                                      value_range=(1, 50), 
                                                                      manager=self.manager,
                                                                      container=self.option_ui_panel)
        self.prm_ui_options = {
            'title': pygame_gui.elements.UILabel(relative_rect=pygame.Rect(20, 20, 121, 40),
                                                 text='PRM Options',  
                                                 manager=self.manager,
                                                 container=self.option_ui_windows[State.PRM],
                                                 object_id='header'),
            'sample_slider_textbox': pygame_gui.elements.UILabel(relative_rect=pygame.Rect(20, 60, 145, 35), 
                                                                 text=f'Sample Size: {self.prm_options["sample_size"]}',
                                                                 manager=self.manager,
                                                                 container=self.option_ui_windows[State.PRM]),
            'sample_slider': pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect(20, 95, 250, 40),
                                                                    start_value=self.prm_options['sample_size'], 
                                                                    value_range=(100, 1500),
                                                                    manager=self.manager,
                                                                    container=self.option_ui_windows[State.PRM]),
            'neighbour_slider_textbox': pygame_gui.elements.UILabel(relative_rect=pygame.Rect(20, 150, 196, 35),
                                                                    text=f'Connect to {self.prm_options["neighbours"]} neighbours', 
                                                                    manager=self.manager, container=self.option_ui_windows[State.PRM]),
            'neighbour_slider': pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect(20, 185, 250, 40),
                                                                       start_value=self.prm_options['neighbours'], 
                                                                       value_range=(1, 20),
                                                                       manager=self.manager,
                                                                       container=self.option_ui_windows[State.PRM]),
            'set_k': pygame_gui.elements.UIButton(relative_rect=pygame.Rect(20,235,250,40), text='Set Neighbours', manager=self.manager, container=self.option_ui_windows[State.PRM]),
            'search_textbox': pygame_gui.elements.UILabel(relative_rect=pygame.Rect(20, 285, 60, 40), # the drop down menu keeps moving the label ???
                                                          text='Search',
                                                          manager=self.manager,
                                                          container=self.option_ui_windows[State.PRM]),
            'search_options': pygame_gui.elements.UIDropDownMenu(self.searches, self.default_search,
                                                                 pygame.Rect(20, 320, 250, 40), manager=self.manager,
                                                                 container=self.option_ui_windows[State.PRM])
        }
        self.rrt_ui_options = {
            'title': pygame_gui.elements.UILabel(relative_rect=pygame.Rect(20, 20, 105, 40),
                                                 text='RRT Options',
                                                 manager=self.manager,
                                                 container=self.option_ui_windows[State.RRT]),
            'bias_slider_textbox': pygame_gui.elements.UILabel(relative_rect=pygame.Rect(20, 60, 90, 35),
                                                               text=f'Bias: {self.rrt_options["bias"]}', 
                                                               manager=self.manager,
                                                               container=self.option_ui_windows[State.RRT]),
            'bias_slider': pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect(20, 95, 250, 40),
                                                                  start_value=0.1, 
                                                                  value_range=(0, 1),
                                                                  manager=self.manager,
                                                                  container=self.option_ui_windows[State.RRT])
        }
        self.visualize_button = pygame_gui.elements.UIButton(relative_rect=pygame.Rect(20, 490, 250, 60), 
                                                             text="Simulate!",
                                                             manager=self.manager, 
                                                             container=self.option_ui_panel,
                                                             object_id='simulate')

        self.pf_ui_options = {
            'title': pygame_gui.elements.UITextBox('Potential Field Options', pygame.Rect(11, 17, 268, 61), manager=self.manager,
                                                   container=self.option_ui_windows[State.PF]),
            'virtual_textbox': pygame_gui.elements.UITextBox(f'Virtual Potential Field: {self.pf_options["virtual"]}',
                                                                   pygame.Rect(11, 110, 268, 40), manager=self.manager,
                                                                   container=self.option_ui_windows[State.PF]),
            'virtual_button': pygame_gui.elements.UIButton(pygame.Rect(10, 150, 270, 40), "Enable/Disable Virtual Field",
                                         manager=self.manager, container=self.option_ui_windows[State.PF])
        }

        self.change_state(self.default_planner)
        self._running = True

        self.init_state()

    def init_state(self):
        if self.state == State.PRM:
            self.planner = ProbabilisticRoadmap(self.map_size, self.start_pose, self.start_radius, self.goal_pose,
                                                self.goal_radius, self.obstacles,
                                                self.prm_options['neighbours'])
            self.search = AStar(self.planner.nodes)

            self.t = threading.Thread(target=self.planner.sample, args=(self.prm_options['sample_size'],))
            self.t.start()
        elif self.state == State.RRT:
            self.planner = RRT(self.map_size, self.start_pose, self.start_radius, self.goal_pose,
                               self.goal_radius, self.obstacles,
                               self.rrt_options['bias'])
        elif self.state == State.PF:
            self.obstacles = []
            self.planner = PotentialField(self.map_size, self.start_pose, self.start_radius, self.goal_pose, self.goal_radius, self.obstacles, self.map, self.pf_options['virtual'])
            self.t = threading.Thread(target=self.planner.start)
            self.t.start()

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
                self._rect_start_pos = None
                self.add_obstacle(rect)

            elif (self.flags['drag_start']):
                self.flags['drag_start'] = False
                collision = False
                for obs in self.obstacles:
                    if obs.collidepoint(self.transition_pose):
                        collision = True
                        break
                if not collision:
                    self.start_pose = self.transition_pose
                    self.update_pose()
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
                    self.update_pose()
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
                if event.ui_element == self.prm_ui_options['search_options']:
                    self.change_search(event.text)
            if event.user_type == pygame_gui.UI_HORIZONTAL_SLIDER_MOVED:
                if event.ui_element == self.obstacle_slider:
                    self.num_obstacles = event.value
                    self.obstacle_textbox.kill()
                    self.obstacle_textbox = pygame_gui.elements.UILabel(relative_rect=pygame.Rect(20, 60, 180, 35),
                                                                        text=f'Generate {self.num_obstacles} Obstacles',
                                                                        manager=self.manager,
                                                                        container=self.option_ui_panel)

                if event.ui_element == self.rrt_ui_options['bias_slider']:
                    self.rrt_options['bias'] = round(event.value, 2)
                    self.rrt_ui_options['bias_slider_textbox'].kill()
                    self.rrt_ui_options['bias_slider_textbox'] = pygame_gui.elements.UILabel(relative_rect=pygame.Rect(20, 60, 90, 35),
                        text=f'Bias: {self.rrt_options["bias"]}', 
                        manager=self.manager,
                        container=self.option_ui_windows[State.RRT])
                    self.planner.set_bias(self.rrt_options['bias'])

                if event.ui_element == self.prm_ui_options['sample_slider']:
                    self.prm_options['sample_size'] = event.value
                    self.prm_ui_options['sample_slider_textbox'].kill()
                    self.prm_ui_options['sample_slider_textbox'] = pygame_gui.elements.UILabel(relative_rect=pygame.Rect(20, 60, 145, 35),
                        text=f'Sample Size: {self.prm_options["sample_size"]}', 
                        manager=self.manager,
                        container=self.option_ui_windows[State.PRM])
                    self.update_prm_samples()

                if event.ui_element == self.prm_ui_options['neighbour_slider']:
                    self.prm_options['neighbours'] = event.value
                    self.prm_ui_options['neighbour_slider_textbox'].kill()
                    self.prm_ui_options['neighbour_slider_textbox'] = pygame_gui.elements.UILabel(relative_rect=pygame.Rect(20, 150, 196, 35),
                        text=f'Connect to {self.prm_options["neighbours"]} neighbours', 
                        manager=self.manager,
                        container=self.option_ui_windows[State.PRM])
            if event.user_type == pygame_gui.UI_BUTTON_PRESSED:
                if event.ui_element == self.toolbar_buttons['add_obs']:
                    self.flags['set_obs'] = True
                if event.ui_element == self.toolbar_buttons['generate_obs']:
                    self.generate_obstacles()
                if event.ui_element == self.toolbar_buttons['reset_obs']:
                    self.resetObstacles()
                if event.ui_element == self.visualize_button:
                    self.simulateState()
                if event.ui_element == self.prm_ui_options['set_k']:
                    self.planner.update_k(self.prm_options['neighbours'])
                    self.search.path = []
                if event.ui_element == self.pf_ui_options['virtual_button']:
                    self.pf_options['virtual'] = not self.pf_options['virtual']
                    self.pf_ui_options['virtual_textbox'].kill()
                    self.pf_ui_options['virtual_textbox'] = pygame_gui.elements.UITextBox(f'Virtual Potential Field: {self.pf_options["virtual"]}',
                                                                   pygame.Rect(11, 110, 268, 40), manager=self.manager,
                                                                   container=self.option_ui_windows[State.PF])
                    self.planner.virtual = self.pf_options['virtual']
                    self.planner.updated = True

    def on_loop(self):
        self.dt = self.clock.tick(60) / 1000
        self.manager.update(self.dt)
        if (self.flags['drag_start'] or self.flags['drag_goal']):
            pos = pygame.mouse.get_pos()
            self.transition_pose = (pos[0] - self.map_pos[0], pos[1] - self.map_pos[1])

        self.sx, self.sy = self.start_pose
        self.gx, self.gy = self.goal_pose

    def on_render(self):
        self._display_surf.fill("#45494e")
        self.map.fill("#303136")

        pygame.draw.circle(self.map, Color.GREEN, self.start_pose, self.start_radius)
        pygame.draw.circle(self.map, Color.GREEN, self.goal_pose, self.goal_radius)

        if self.transition_pose:
            pygame.draw.circle(self.map, Color.RED, self.transition_pose, 15)

        if self._rect_start_pos:
            curr_pos = localize(self.map_pos, pygame.mouse.get_pos())
            w = curr_pos[0] - self._rect_start_pos[0]
            h = curr_pos[1] - self._rect_start_pos[1]
            rect = pygame.Rect(*self._rect_start_pos, w, h)
            rect.normalize()
            pygame.draw.rect(self.map, Color.BLUE, rect)

        self.renderState()
        self.manager.draw_ui(self._display_surf)

        pygame.display.update()

    def on_cleanup(self):
        pygame.quit()
        sys.exit()

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
            if self.state != state:
                if self.state == State.PF:
                    self.obstacles = []
                self.state = State.PRM
                self.init_state()
        elif state == 'RRT':
            if self.state != state:
                if self.state == State.PF:
                    self.obstacles = []
                self.state = State.RRT
                self.init_state()
        elif state == 'Potential Field':
            if self.state != state:
                self.state = State.PF
                self.init_state()

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

    def change_search(self, search):
        if search == 'Dijkstra' and self.search is not Dijkstra:
            path = self.search.path
            self.search = Dijkstra(self.planner.nodes)
            self.search.path = path
        elif search == 'A*' and self.search is not AStar:
            path = self.search.path
            self.search = AStar(self.planner.nodes)
            self.search.path = path
        elif search == 'Greedy Best First' and self.search is not GreedyBFS:
            path = self.search.path
            self.search = GreedyBFS(self.planner.nodes)
            self.search.path = path
        for node in self.planner.nodes:
            node.parent = None
            if node.search == self.search.name:
                node.search = None

    def update_prm_samples(self):
        self.planner.set_obstacles(self.obstacles)
        self.search.path = []
        self.t = threading.Thread(target=self.planner.sample, args=(self.prm_options['sample_size'],))
        self.t.start()

    def generate_obstacles(self):
        if self.state == State.PRM:
            self.obstacles = generate_obs(self.num_obstacles, self.map_pos, self.map_size, self.obs_dim)
            self.planner.set_obstacles(self.obstacles)
            self.planner.nodes = []

            self.search.path = []
            self.t = threading.Thread(target=self.planner.sample, args=(self.prm_options['sample_size'],))
            self.t.start()
        elif self.state == State.RRT:
            self.obstacles = generate_obs(self.num_obstacles, self.map_pos, self.map_size, self.obs_dim)
            self.planner.set_obstacles(self.obstacles)
            self.planner.nodes = []

        elif self.state == State.PF:
            self.obstacles = generate_circle_obs(self.num_obstacles, self.map_pos, self.map_size, self.circle_obs_dim, self.goal_pose)
            self.planner.set_obstacles(self.obstacles)
            self.planner.updated = True

    def add_obstacle(self, rect):
        self.obstacles.append(rect)
        self.planner.set_obstacles(self.obstacles)
        if self.state == State.PRM:
            self.t = threading.Thread(target=self.planner.sample, args=(self.prm_options['sample_size'],))
            self.t.start()
        elif self.state == State.RRT:
            pass
        elif self.state == State.PF:
            pass

    def renderState(self):
        if self.state == State.PRM:
            for obj in self.obstacles:
                pygame.draw.rect(self.map, Color.LIGHT_PURPLE, obj)

            for node in self.planner.nodes:
                node.draw(self.map, self.node_radius, 1)

            for node in self.search.path:
                pygame.draw.circle(self.map, Color.GREEN, node.get_coords(), self.node_radius + 2, width=0)

        if self.state == State.RRT:
            for obj in self.obstacles:
                pygame.draw.rect(self.map, Color.LIGHT_PURPLE, obj)

            for node in self.planner.nodes:
                node.draw(self.map, self.node_radius, 1)

            for node in self.planner.path:
                pygame.draw.circle(self.map, Color.RED, node.get_coords(), self.node_radius + 2, width=0)

        if self.state == State.PF:
            for obs in self.obstacles:
                pygame.draw.circle(self.map, Color.LIGHT_PURPLE, (obs.x,obs.y), obs.rad, width=0)
            self.planner.draw(self.map)
            for node in self.planner.path:
                pygame.draw.circle(self.map, Color.LIGHT_BLUE, node.get_coords(), self.node_radius, width=0)

    def simulateState(self):
        if self.state == State.PRM:
            self.planner.create_network(self.map, self.node_radius, self.prm_options['neighbours'])
            if self.t is None or not self.t.is_alive():
                self.t = threading.Thread(target=self.search.solve, args=(
                    self.planner.nodes, self.planner.get_start_node(), self.planner.get_end_node(),))
                self.t.start()
        if self.state == State.RRT:
            if self.t is None or not self.t.is_alive():
                self.t = threading.Thread(target=self.planner.start, daemon=True)
                self.t.start()
        if self.state ==  State.PF:
            self.planner.updated = True
            if self.t is None or not self.t.is_alive():
                self.t = threading.Thread(target=self.planner.start, daemon=True)
                self.t.start()
            else:
                self.t.join()
                self.t = threading.Thread(target=self.planner.start, daemon=True)
                self.t.start()

    def update_pose(self):
        self.planner.update_pose(self.start_pose, self.goal_pose)
        if self.state == State.PRM:
            self.t = threading.Thread(target=self.search.update_solution,
                                      args=(self.planner.get_start_node(), self.planner.get_end_node(),))
            self.t.start()
        elif self.state == State.RRT:
            if self.t is None or not self.t.is_alive():
                self.t = threading.Thread(target=self.planner.start, daemon=True)
                self.t.start()
        elif self.state == State.PF:
            self.planner.updated = True
            if self.t is None or not self.t.is_alive():
                self.t = threading.Thread(target=self.planner.start, daemon=True)
                self.t.start()
            else:
                self.t.join()
                self.t = threading.Thread(target=self.planner.start, daemon=True)
                self.t.start()
    def resetObstacles(self):
        if self.state == State.PRM:
            self.planner.obstacles = []
            self.obstacles = []
            self.search.path = []
        elif self.state == State.RRT:
            self.planner.obstacles = []
            self.obstacles = []
            self.planner.path = []


if __name__ == "__main__":
    theApp = App()
    theApp.on_execute()
