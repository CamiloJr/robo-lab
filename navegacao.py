"""Nucleo independente de interface para robotica movel didatica.

A*: planejamento global em grade (modelo conhecido).
DWA simplificado: avaliacao de trajetorias cinematicas curtas.
Q-learning: aprendizado por reforco tabular, separado da navegacao em tempo real.
LiDAR: intersecoes geometricas simuladas, sem equipamento fisico.

Unidades: pixel de mundo ~= unidade espacial simulada; segundos.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import heapq
import math
import random
from collections import deque

WIDTH, HEIGHT, CELL = 1000, 620, 25
COLS, ROWS = WIDTH // CELL, math.ceil(HEIGHT / CELL)
ROBOT_R = 14.0
TAU = math.tau


def clamp(v, low, high):
    return max(low, min(high, v))


def norm_angle(a):
    return (a + math.pi) % TAU - math.pi


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


@dataclass
class Obstacle:
    x: float
    y: float
    w: float = 0
    h: float = 0
    r: float = 0
    name: str = 'Obstaculo'
    kind: str = 'static'   # static, user

    @property
    def is_circle(self):
        return self.r > 0


@dataclass
class Actor:
    x: float
    y: float
    ax: float
    ay: float
    bx: float
    by: float
    speed: float = 55.0
    radius: float = 18.0
    phase: float = 0.0

    def update(self, dt):
        dx, dy = self.bx - self.ax, self.by - self.ay
        length = math.hypot(dx, dy)
        if length < 1:
            return
        self.phase += self.speed * dt / length
        # Triangular movement; velocity reverses at endpoints
        u = self.phase % 2.0
        t = u if u <= 1 else 2 - u
        self.x = self.ax + dx * t
        self.y = self.ay + dy * t

    def velocity(self):
        dx, dy = self.bx - self.ax, self.by - self.ay
        length = math.hypot(dx, dy) or 1
        sign = 1 if self.phase % 2 < 1 else -1
        return self.speed * dx / length * sign, self.speed * dy / length * sign


def clearance(x, y, obstacles, actors=()):
    """Distancia assinada ate o obstaculo mais proximo, incluindo limites."""
    d = min(x, y, WIDTH - x, HEIGHT - y)
    for o in obstacles:
        if o.is_circle:
            d = min(d, math.hypot(x - o.x, y - o.y) - o.r)
        else:
            dx = max(o.x - x, 0, x - (o.x + o.w))
            dy = max(o.y - y, 0, y - (o.y + o.h))
            outside = math.hypot(dx, dy)
            if dx == 0 and dy == 0:
                outside = -min(x - o.x, o.x + o.w - x, y - o.y, o.y + o.h - y)
            d = min(d, outside)
    for a in actors:
        d = min(d, math.hypot(x - a.x, y - a.y) - a.radius)
    return d


def occupied(x, y, obstacles, actors=(), inflate=ROBOT_R + 8):
    return clearance(x, y, obstacles, actors) < inflate


def make_scenario(name):
    if name == 'resgate':
        obs = [
            Obstacle(200, 105, 80, 230, name='Parede'),
            Obstacle(200, 420, 190, 70, name='Parede'),
            Obstacle(400, 95, 65, 270, name='Parede'),
            Obstacle(610, 270, 75, 260, name='Parede'),
            Obstacle(770, 90, 60, 150, name='Parede'),
            Obstacle(350, 530, r=30, name='Entulho'),
            Obstacle(530, 155, r=37, name='Entulho'),
            Obstacle(860, 420, r=30, name='Entulho'),
        ]
        act = [Actor(500, 425, 480, 425, 585, 425, 30, 17),
               Actor(720, 155, 710, 155, 735, 255, 28, 17)]
        return (90, 540), (915, 80), obs, act
    if name == 'cidade':
        obs = [
            Obstacle(155, 85, 165, 130, name='Edificio'),
            Obstacle(155, 345, 165, 165, name='Edificio'),
            Obstacle(435, 80, 145, 135, name='Edificio'),
            Obstacle(435, 345, 145, 160, name='Edificio'),
            Obstacle(730, 90, 135, 125, name='Edificio'),
            Obstacle(730, 355, 135, 145, name='Edificio'),
            Obstacle(380, 260, r=24, name='Rotatoria'),
            Obstacle(685, 265, r=23, name='Rotatoria'),
        ]
        act = [Actor(80, 278, 80, 278, 915, 278, 105, 17, 0.21),
               Actor(625, 70, 625, 70, 625, 535, 75, 17, 0.42),
               Actor(360, 540, 360, 540, 360, 80, 55, 17, 0.60)]
        return (80, 550), (918, 68), obs, act
    obs = [
        Obstacle(245, 78, 100, 110, name='Prateleira'),
        Obstacle(245, 275, 100, 108, name='Prateleira'),
        Obstacle(245, 472, 100, 85, name='Prateleira'),
        Obstacle(485, 76, 100, 112, name='Prateleira'),
        Obstacle(485, 275, 100, 108, name='Prateleira'),
        Obstacle(485, 472, 100, 85, name='Prateleira'),
        Obstacle(755, 75, 95, 110, name='Prateleira'),
        Obstacle(755, 275, 95, 108, name='Prateleira'),
        Obstacle(755, 472, 95, 85, name='Prateleira'),
    ]
    act = [Actor(90, 226, 90, 226, 910, 226, 76, 17, .11),
           Actor(870, 420, 100, 420, 890, 420, 62, 17, .89),
           Actor(420, 500, 420, 60, 420, 548, 49, 16, .38)]
    return (78, 540), (918, 90), obs, act


class Grid:
    def __init__(self, obstacles, actors=(), inflate=ROBOT_R + 10):
        self.blocked = set()
        self.inflate = inflate
        for y in range(ROWS):
            for x in range(COLS):
                px, py = self.center((x, y))
                if occupied(px, py, obstacles, actors, inflate):
                    self.blocked.add((x, y))

    @staticmethod
    def cell(pos):
        return (int(clamp(pos[0] // CELL, 0, COLS - 1)),
                int(clamp(pos[1] // CELL, 0, ROWS - 1)))

    @staticmethod
    def center(c):
        return ((c[0] + .5) * CELL, min((c[1] + .5) * CELL, HEIGHT - 1))

    def free(self, cell):
        return 0 <= cell[0] < COLS and 0 <= cell[1] < ROWS and cell not in self.blocked

    def nearest_free(self, cell):
        if self.free(cell):
            return cell
        x, y = cell
        for radius in range(1, max(COLS, ROWS)):
            candidates = [(i, j) for i in range(x - radius, x + radius + 1)
                          for j in range(y - radius, y + radius + 1)
                          if max(abs(x-i), abs(y-j)) == radius and self.free((i, j))]
            if candidates:
                return min(candidates, key=lambda p: (p[0]-x)**2+(p[1]-y)**2)
        return None

    def neighbors(self, c, diagonal=True):
        x, y = c
        delta = ((1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)) if diagonal else ((1,0),(-1,0),(0,1),(0,-1))
        for dx, dy in delta:
            n = (x+dx, y+dy)
            if not self.free(n):
                continue
            if dx and dy and (not self.free((x+dx, y)) or not self.free((x, y+dy))):
                continue
            yield n, math.hypot(dx, dy)


def astar(grid, start, goal):
    start, goal = grid.nearest_free(start), grid.nearest_free(goal)
    if start is None or goal is None:
        return [], set()
    frontier = [(0, 0, start)]
    seq = 1
    best = {start: 0.0}
    parents = {}
    visited = set()
    while frontier:
        _, _, current = heapq.heappop(frontier)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            path = [current]
            while path[-1] != start:
                path.append(parents[path[-1]])
            path.reverse()
            return path, visited
        for nei, cost in grid.neighbors(current):
            alt = best[current] + cost
            if alt < best.get(nei, math.inf):
                best[nei] = alt
                parents[nei] = current
                heuristic = math.hypot(goal[0] - nei[0], goal[1] - nei[1])
                heapq.heappush(frontier, (alt + heuristic, seq, nei))
                seq += 1
    return [], visited


def has_sight(p1, p2, obstacles, actors=(), inflate=ROBOT_R + 6):
    length = dist(p1, p2)
    steps = max(1, math.ceil(length / 8))
    for i in range(steps + 1):
        t = i/steps
        if occupied(p1[0] + (p2[0]-p1[0])*t,
                    p1[1] + (p2[1]-p1[1])*t, obstacles, actors, inflate):
            return False
    return True


def smooth_path(path, obstacles, actors=(), start=None, goal=None):
    if not path:
        return []
    pts = [Grid.center(cell) for cell in path]
    if start is not None:
        pts[0] = tuple(start)
    if goal is not None and has_sight(pts[-1], goal, obstacles, actors):
        pts[-1] = tuple(goal)
    result = [pts[0]]
    i = 0
    while i < len(pts)-1:
        next_i = i + 1
        for j in range(len(pts)-1, i, -1):
            if has_sight(pts[i], pts[j], obstacles, actors):
                next_i = j
                break
        result.append(pts[next_i])
        i = next_i
    return result


def ray_aabb(x, y, dx, dy, ob):
    low, high = -math.inf, math.inf
    for start, direction, a, b in ((x, dx, ob.x, ob.x+ob.w),
                                   (y, dy, ob.y, ob.y+ob.h)):
        if abs(direction) < 1e-9:
            if not a <= start <= b:
                return math.inf
        else:
            t1, t2 = (a-start)/direction, (b-start)/direction
            low = max(low, min(t1, t2))
            high = min(high, max(t1, t2))
            if high < low:
                return math.inf
    return low if low >= 0 else (0 if high >= 0 else math.inf)


def ray_circle(x, y, dx, dy, cx, cy, r):
    ox, oy = x-cx, y-cy
    b = ox*dx + oy*dy
    c = ox*ox + oy*oy - r*r
    disc = b*b - c
    if disc < 0:
        return math.inf
    t = -b - math.sqrt(disc)
    if t >= 0:
        return t
    return 0 if c <= 0 else math.inf


def lidar_scan(x, y, theta, obstacles, actors, beams=72, max_range=190, span=math.radians(270)):
    result = []
    for i in range(beams):
        angle = theta - span/2 + span * i / max(1, beams-1)
        dx, dy = math.cos(angle), math.sin(angle)
        distance = max_range
        for o in obstacles:
            t = (ray_circle(x,y,dx,dy,o.x,o.y,o.r) if o.is_circle
                 else ray_aabb(x,y,dx,dy,o))
            distance = min(distance, t)
        for a in actors:
            distance = min(distance, ray_circle(x,y,dx,dy,a.x,a.y,a.radius))
        for edge, pos, d in ((0,x,dx),(WIDTH,x,dx),(0,y,dy),(HEIGHT,y,dy)):
            if d:
                t = (edge - pos) / d
                if t >= 0:
                    distance = min(distance,t)
        result.append((angle, max(0.0, distance)))
    return result


@dataclass
class Candidate:
    v: float
    w: float
    score: float
    positions: list[tuple[float,float]]
    min_clear: float


def dwa_candidates(x, y, theta, speed, turn, waypoint, obstacles, actors,
                   max_speed=125.0, safety=8.0):
    """Busca local em velocidades (DWA didatico, sem implementacao ROS)."""
    v_grid = [0, 0.22, 0.43, 0.65, 0.85, 1.0]
    w_grid = [-2.7,-2.0,-1.4,-.9,-.45,0,.45,.9,1.4,2.0,2.7]
    horizon, steps = 0.90, 9
    candidates = []
    for v_frac in v_grid:
        v = v_frac * max_speed
        for w in w_grid:
            px, py, angle = x, y, theta
            points = [(px,py)]
            min_clear = math.inf
            collision = False
            for step in range(steps):
                tau = horizon / steps
                angle = norm_angle(angle + w * tau)
                px += math.cos(angle)*v*tau
                py += math.sin(angle)*v*tau
                points.append((px,py))
                # Actors predicted linearly; obstacle assessment is local, approximate.
                dynamic = []
                for a in actors:
                    vx,vy = a.velocity()
                    dynamic.append(Actor(a.x+vx*tau*(step+1), a.y+vy*tau*(step+1), 0,0,0,0,
                                         radius=a.radius))
                c = clearance(px,py,obstacles,dynamic)
                min_clear = min(c, min_clear)
                if c < ROBOT_R + safety:
                    collision = True
                    break
            if collision:
                continue
            goal_dist = dist((px,py), waypoint)
            desired = math.atan2(waypoint[1]-py, waypoint[0]-px)
            heading = math.cos(norm_angle(desired-angle))
            safe_norm = clamp((min_clear-ROBOT_R-safety)/110,0,1)
            # Incentive to approach the local target, align, move, and avoid risks.
            score = (-.025*goal_dist + 1.32*heading +
                     .65*v_frac*max(0.0,heading) + .62*safe_norm - .040*abs(w))
            candidates.append(Candidate(v,w,score,points,min_clear))
    candidates.sort(key=lambda c:c.score,reverse=True)
    return candidates


class QLearner:
    """Q-learning discreto em grade, politica experimental (nao controla o DWA)."""
    ACTIONS = ((1,0),(-1,0),(0,1),(0,-1))

    def __init__(self, grid, goal, seed=42):
        self.grid = grid
        self.goal = grid.nearest_free(goal)
        self.random = random.Random(seed)
        self.q = {}
        self.episodes = 0
        self.successes = deque(maxlen=100)
        self.returns = deque(maxlen=100)
        self.history = []
        self.target_episodes = 2400
        self.finished = False
        self.free_cells = [(x,y) for y in range(ROWS) for x in range(COLS) if grid.free((x,y))]

    def values(self, cell):
        if cell not in self.q:
            self.q[cell] = [0.0]*4
        return self.q[cell]

    def train(self, batch=12):
        if not self.goal or not self.free_cells or self.finished:
            return
        for _ in range(batch):
            if self.episodes >= self.target_episodes:
                self.finished = True
                break
            eps = max(.045, 1.0 - .94*self.episodes/self.target_episodes)
            st = self.random.choice(self.free_cells)
            total, success = 0, False
            for step in range(150):
                if st == self.goal:
                    success = True
                    break
                actions = self.values(st)
                if self.random.random() < eps:
                    action = self.random.randrange(4)
                else:
                    action = self.random.choice([i for i,v in enumerate(actions) if v == max(actions)])
                dx,dy = self.ACTIONS[action]
                proposed = (st[0]+dx,st[1]+dy)
                valid = self.grid.free(proposed)
                ne = proposed if valid else st
                reached = ne == self.goal
                # Reward shaping, only geometry: no knowledge of a planned route.
                oldd = math.hypot(st[0]-self.goal[0],st[1]-self.goal[1])
                newd = math.hypot(ne[0]-self.goal[0],ne[1]-self.goal[1])
                reward = (125 if reached else -1.0 + 1.8*(oldd-newd) - (7 if not valid else 0))
                arr = self.values(st)
                future = 0.0 if reached else max(self.values(ne))
                arr[action] += .36*(reward + .96*future - arr[action])
                total += reward
                st = ne
                if reached:
                    success = True
                    break
            self.episodes += 1
            self.successes.append(int(success))
            self.returns.append(total)
            if self.episodes % 25 == 0:
                self.history.append((self.episodes, sum(self.successes)/len(self.successes)))

    def learned_path(self, start, limit=400):
        if not self.goal or self.episodes < 25:
            return []
        st = self.grid.nearest_free(start)
        seen = set()
        route = [st]
        for _ in range(limit):
            if st == self.goal:
                return route
            if st in seen:
                return []
            seen.add(st)
            vals = self.q.get(st, [0.0]*4)
            ix = max(range(4),key=lambda i: vals[i])
            dx,dy = self.ACTIONS[ix]
            nxt = (st[0]+dx,st[1]+dy)
            if not self.grid.free(nxt):
                return []
            route.append(nxt)
            st = nxt
        return []

    @property
    def success_rate(self):
        return (100*sum(self.successes)/len(self.successes)) if self.successes else 0.0


class Simulator:
    def __init__(self):
        self.show_lidar = True
        self.show_path = True
        self.show_search = False
        self.show_q = True
        self.show_heat = False
        self.dynamic_enabled = True
        self.autopilot = True
        self.max_speed = 125
        self.lidar_range = 190
        self.safety = 8
        self.is_paused = False
        self.scenario = 'armazem'
        self.reset('armazem')

    def reset(self, scene=None):
        if scene:
            self.scenario = scene
        self.start, self.goal, self.obstacles, self.actors = make_scenario(self.scenario)
        self.x, self.y = self.start
        self.theta = -.68
        self.v = self.w = 0.0
        self.time = 0.0
        self.travelled = 0.0
        self.collisions_avoided = 0
        self.replans = 0
        self.last_plan_t = -100.0
        self.last_lidar_t = -100.0
        self.last_dwa_t = -100.0
        self.scan = []
        self.min_lidar = self.lidar_range
        self.reached = False
        self.message = 'Inicializando percepcao e planejamento.'
        self.path = []
        self.raw_path = []
        self.visited = set()
        self.dwa = []
        self.chosen = None
        self.path_target = 0
        self.waypoint = self.goal
        self.trail = deque(maxlen=500)
        self.last_stop = -100
        self.score_log = deque(maxlen=120)
        self.grid_static = Grid(self.obstacles)
        self.qlearn = QLearner(self.grid_static, Grid.cell(self.goal))
        self.learning = False
        self.plan_route(force=True)
        self.update_lidar()

    def plan_route(self,force=False):
        if self.reached:
            return
        # A* opera sobre o mapa estatico; agentes moveis sao tratados pelo DWA.
        # Esta separacao evita que mudancas rapidas causem oscilacao global de rotas.
        actors = []
        grid = Grid(self.obstacles, actors, ROBOT_R+10)
        start_cell = Grid.cell((self.x,self.y))
        dest_cell = Grid.cell(self.goal)
        raw, visited = astar(grid, start_cell, dest_cell)
        self.raw_path = raw
        self.visited = visited
        self.path = smooth_path(raw,self.obstacles,actors,(self.x,self.y),self.goal)
        self.path_target = 1 if len(self.path)>1 else 0
        self.last_plan_t = self.time
        self.replans += 1
        self.message = ('Rota A* recalculada: {} celulas exploradas.'.format(len(visited))
                        if raw else 'A*: nenhuma rota segura. Remova algum obstaculo.')

    def set_goal(self,x,y):
        if occupied(x,y,self.obstacles,(), ROBOT_R+15):
            self.message = 'Destino invalido: escolha uma area livre.'
            return False
        self.goal = (x,y)
        self.reached = False
        self.last_plan_t = -100
        self.qlearn = QLearner(self.grid_static,Grid.cell(self.goal))
        self.learning = False
        self.plan_route(force=True)
        return True

    def add_obstacle(self,x,y):
        if occupied(x,y,self.obstacles,(),45) or dist((x,y),(self.x,self.y)) < 70 or dist((x,y),self.goal)<55:
            self.message = 'Nao e possivel inserir obstaculo aqui.'
            return False
        self.obstacles.append(Obstacle(x-20,y-20,40,40,name='Caixa',kind='user'))
        self.grid_static = Grid(self.obstacles)
        self.qlearn = QLearner(self.grid_static,Grid.cell(self.goal))
        self.learning = False
        self.plan_route(force=True)
        return True

    def remove_obstacle(self,x,y):
        user = [o for o in self.obstacles if o.kind == 'user']
        if not user:
            self.message = 'Nao ha caixas adicionadas para remover.'
            return False
        target = min(user,key=lambda o:dist((o.x+o.w/2,o.y+o.h/2),(x,y)))
        if dist((target.x+target.w/2,target.y+target.h/2),(x,y))>60:
            return False
        self.obstacles.remove(target)
        self.grid_static = Grid(self.obstacles)
        self.qlearn = QLearner(self.grid_static,Grid.cell(self.goal))
        self.learning = False
        self.plan_route(force=True)
        return True

    def update_lidar(self):
        actors = self.actors if self.dynamic_enabled else []
        self.scan = lidar_scan(self.x,self.y,self.theta,self.obstacles,actors,max_range=self.lidar_range)
        self.min_lidar = min((d for _,d in self.scan),default=self.lidar_range)
        self.last_lidar_t = self.time

    def get_waypoint(self):
        if not self.path:
            return None
        while self.path_target < len(self.path)-1 and dist((self.x,self.y),self.path[self.path_target]) < 48:
            self.path_target += 1
        if self.path_target >= len(self.path):
            return self.goal
        # 1-step lookahead helps the local controller turn in time
        return self.path[self.path_target]

    def step(self,dt,manual=(0,0)):
        if self.is_paused:
            return
        dt = clamp(dt,0,0.05)
        self.time += dt
        if self.dynamic_enabled:
            for a in self.actors:
                old=(a.x,a.y,a.phase)
                a.update(dt)
                # AGVs simulados possuem parada de seguranca cooperativa.
                # Impede que outro agente atraves um robo imobilizado.
                if dist((a.x,a.y),(self.x,self.y))<a.radius+ROBOT_R+8:
                    a.x,a.y=old[0],old[1]
                    # Reversao de sentido na mesma posicao da rota triangular.
                    a.phase=2.0-(old[2] % 2.0)
        if self.learning:
            self.qlearn.train(14)
            if self.qlearn.finished:
                self.learning = False
        if self.time-self.last_lidar_t>.09:
            self.update_lidar()
        if not self.reached and dist((self.x,self.y),self.goal)<33:
            self.reached=True
            self.v=self.w=0
            self.message='MISSAO CONCLUIDA! O destino foi alcancado.'
        if self.reached:
            self.v = self.w = 0
            return
        # Local planner reacts to dynamic obstacles by adding their positions to A*.
        if self.autopilot and self.time-self.last_plan_t>1.05:
            self.plan_route()
        if self.autopilot:
            wp = self.get_waypoint()
            if wp is None:
                target_v = target_w = 0.0
                self.chosen = None
                self.dwa = []
            else:
                self.waypoint = wp
                if self.time-self.last_dwa_t>.18 or self.chosen is None:
                    self.dwa = dwa_candidates(self.x,self.y,self.theta,self.v,self.w,wp,self.obstacles,
                        self.actors if self.dynamic_enabled else [],max_speed=self.max_speed,safety=self.safety)
                    # Recuperacao: se preso num corredor estreito, reduza apenas
                    # a margem EXTRA de seguranca, nunca o raio do robo.
                    if not self.dwa:
                        for margin in (4.0,1.0,0.0):
                            if margin < self.safety:
                                self.dwa = dwa_candidates(self.x,self.y,self.theta,self.v,self.w,wp,
                                    self.obstacles,self.actors if self.dynamic_enabled else [],
                                    max_speed=min(75,self.max_speed),safety=margin)
                            if self.dwa:
                                self.message='Recuperacao: avancar com margem adicional reduzida.'
                                break
                    self.chosen = self.dwa[0] if self.dwa else None
                    self.last_dwa_t = self.time
                    if not self.dwa:
                        self.message = 'Controlador local: sem trajetoria livre; parando.'
                target_v = self.chosen.v if self.chosen else 0
                # Em uma travessia temporariamente bloqueada, o robo ainda
                # pode girar no lugar para buscar uma nova rota viavel.
                target_w = (self.chosen.w if self.chosen else
                            clamp(3*norm_angle(math.atan2(wp[1]-self.y,wp[0]-self.x)-self.theta),-2.7,2.7))
                if dist((self.x,self.y),self.goal)<55:
                    target_v = min(target_v, max(20,dist((self.x,self.y),self.goal)*1.6))
        else:
            fwd,steer=manual
            target_v = fwd * self.max_speed*.85
            target_w = steer*2.6
            self.dwa=[]
            self.chosen=None
        # Differential-drive kinematics; acceleration / angular acceleration limits
        # Freio reflexo baseado nos retornos LiDAR do setor frontal.
        # O planejador A* conhece o mapa; esta camada sensorial atua no controle.
        if target_v > 0 and self.scan:
            forward_rays=[rng for angle,rng in self.scan
                          if abs(norm_angle(angle-self.theta)) < .16]
            if forward_rays:
                nearest=min(forward_rays)
                # Freio apenas na proximidade critica; o DWA ja preve
                # distancia de parada e desvio pela geometria conhecida.
                if nearest < ROBOT_R+12:
                    target_v=min(target_v,max(0.0,(nearest-ROBOT_R-4)*6.0))
        self.v += clamp(target_v-self.v,-230*dt,230*dt)
        self.w += clamp(target_w-self.w,-7*dt,7*dt)
        if abs(self.v)<.3:self.v=0
        if abs(self.w)<.02:self.w=0
        new_theta=norm_angle(self.theta+self.w*dt)
        new_x=self.x + self.v*math.cos(new_theta)*dt
        new_y=self.y + self.v*math.sin(new_theta)*dt
        if occupied(new_x,new_y,self.obstacles,self.actors if self.dynamic_enabled else (),ROBOT_R+2):
            self.v = self.w = 0
            if self.time-self.last_stop>0.7:
                self.collisions_avoided +=1
                self.last_stop=self.time
                self.message = 'Parada de seguranca: colisao prevista no proximo passo.'
                self.last_plan_t=-100
        else:
            self.travelled+=math.hypot(new_x-self.x,new_y-self.y)
            self.x,self.y=new_x,new_y
            self.theta=new_theta
            if not self.trail or dist(self.trail[-1],(self.x,self.y))>3:
                self.trail.append((self.x,self.y))
        if dist((self.x,self.y),self.goal)<33:
            self.reached=True
            self.v=self.w=0
            self.message='MISSAO CONCLUIDA! O destino foi alcancado.'

    def q_path(self):
        return self.qlearn.learned_path(Grid.cell(self.start))

    def toggle_train(self):
        if self.qlearn.finished:
            self.qlearn = QLearner(self.grid_static,Grid.cell(self.goal))
        self.learning = not self.learning
        self.message = ('Q-learning: agente aprendendo com recompensas.' if self.learning
                        else 'Q-learning pausado; politica conservada.')
