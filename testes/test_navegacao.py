"""Testes da IA e simulacao sem carregar pygame: python -m unittest discover -s testes -v."""
import math
import unittest
from navegacao import (
    Simulator,Obstacle,Actor,Grid,QLearner,occupied,clearance,
    ray_circle,ray_aabb,lidar_scan,astar,smooth_path,has_sight,dist,
    ROBOT_R,CELL,WIDTH,HEIGHT,make_scenario,dwa_candidates
)


class TestGeom(unittest.TestCase):
    def test_clearance_circle(self):
        o=Obstacle(100,100,r=25)
        self.assertAlmostEqual(clearance(150,100,[o]),25)
        self.assertLess(clearance(100,100,[o]),0)
    def test_clearance_rectangle(self):
        o=Obstacle(100,100,w=80,h=60)
        self.assertAlmostEqual(clearance(90,120,[o]),10)
        self.assertLess(clearance(110,120,[o]),0)
    def test_ray_aabb(self):
        self.assertAlmostEqual(ray_aabb(0,120,1,0,Obstacle(100,100,80,50)),100)
        self.assertTrue(math.isinf(ray_aabb(0,0,1,0,Obstacle(100,100,80,50))))
    def test_ray_circle(self):
        self.assertAlmostEqual(ray_circle(0,100,1,0,100,100,25),75)
        self.assertTrue(math.isinf(ray_circle(0,100,-1,0,100,100,25)))
    def test_lidar_hits_object(self):
        scan=lidar_scan(50,100,0,[Obstacle(120,80,w=40,h=40)],[],beams=5,max_range=150,span=math.pi)
        self.assertEqual(len(scan),5)
        self.assertAlmostEqual(scan[2][1],70)
    def test_collision_margin(self):
        o=Obstacle(100,100,r=20)
        self.assertTrue(occupied(100,130,[o],inflate=11))
        self.assertFalse(occupied(100,145,[o],inflate=11))


class TestPlan(unittest.TestCase):
    def test_astar_path_in_three_scenarios(self):
        for scene in ('armazem','resgate','cidade'):
            start,goal,obstacles,actors=make_scenario(scene)
            grid=Grid(obstacles,actors)
            path,explored=astar(grid,Grid.cell(start),Grid.cell(goal))
            self.assertGreater(len(path),15)
            self.assertTrue(all(grid.free(c) for c in path))
            self.assertGreater(len(explored),len(path))
    def test_diagonal_does_not_cut_corner(self):
        obs=[Obstacle(52,27,20,20),Obstacle(27,52,20,20)]
        grid=Grid(obs,inflate=1)
        # (1,1) should not cut diagonal to (2,2) when adjacent occupied
        cells=[n for n,_ in grid.neighbors((1,1))]
        self.assertNotIn((2,2),cells)
    def test_smoothing_preserves_clearance(self):
        start,goal,obstacles,actors=make_scenario('armazem')
        grid=Grid(obstacles)
        raw,_=astar(grid,Grid.cell(start),Grid.cell(goal))
        path=smooth_path(raw,obstacles,(),start,goal)
        self.assertLess(len(path),len(raw))
        self.assertTrue(all(has_sight(a,b,obstacles) for a,b in zip(path,path[1:])))
    def test_dwa_selects_non_colliding_trajectory(self):
        obstacles=[Obstacle(200,250,80,120)]
        cand=dwa_candidates(110,310,0,0,0,(500,310),obstacles,[],safety=8)
        self.assertGreater(len(cand),0)
        self.assertGreaterEqual(cand[0].min_clear,ROBOT_R+8)
        self.assertGreaterEqual(cand[0].score,cand[-1].score)


class TestReinforcement(unittest.TestCase):
    def test_q_learning_converges_and_produces_route(self):
        for scene in ('armazem','resgate','cidade'):
            start,goal,obstacles,_=make_scenario(scene)
            q=QLearner(Grid(obstacles),Grid.cell(goal))
            while not q.finished:
                q.train(150)
            self.assertEqual(q.episodes,q.target_episodes)
            self.assertGreater(q.success_rate,85)
            route=q.learned_path(Grid.cell(start))
            self.assertTrue(route,scene)
            self.assertEqual(route[-1],q.goal)
            self.assertTrue(all(q.grid.free(c) for c in route))
    def test_training_pause_is_lossless(self):
        s=Simulator()
        s.toggle_train()
        self.assertTrue(s.learning)
        for _ in range(20):s.step(.025)
        ep=s.qlearn.episodes
        s.toggle_train()
        for _ in range(50):s.step(.025)
        self.assertEqual(ep,s.qlearn.episodes)
    def test_new_goal_resets_learning(self):
        s=Simulator()
        s.qlearn.train(50)
        self.assertTrue(s.set_goal(86,92))
        self.assertEqual(s.qlearn.episodes,0)


class TestSimulator(unittest.TestCase):
    def test_robot_reaches_all_targets(self):
        for scene in ('armazem','resgate','cidade'):
            s=Simulator()
            s.reset(scene)
            for _ in range(55*42):
                s.step(1/55)
                self.assertGreaterEqual(clearance(s.x,s.y,s.obstacles,s.actors if s.dynamic_enabled else ()),ROBOT_R+1)
                if s.reached:
                    break
            self.assertTrue(s.reached, '{} position {} {}'.format(scene,s.x,s.y))
            self.assertLess(s.time,42)
    def test_add_delete_obstacle(self):
        s=Simulator()
        before=len(s.obstacles)
        self.assertTrue(s.add_obstacle(110,360))
        self.assertEqual(len(s.obstacles),before+1)
        self.assertTrue(s.remove_obstacle(110,360))
        self.assertEqual(len(s.obstacles),before)
    def test_cannot_add_over_robot(self):
        s=Simulator()
        self.assertFalse(s.add_obstacle(s.x,s.y))
    def test_cannot_set_goal_on_wall(self):
        s=Simulator()
        self.assertFalse(s.set_goal(290,110))
    def test_pause_preserves_time(self):
        s=Simulator()
        s.is_paused=True
        for _ in range(20):s.step(.04)
        self.assertEqual(s.time,0)
    def test_manual_rotation(self):
        s=Simulator()
        s.autopilot=False
        angle=s.theta
        for _ in range(30):s.step(.02,(0,1))
        self.assertGreater(abs(s.theta-angle),.2)
    def test_dynamic_disable(self):
        s=Simulator()
        s.dynamic_enabled=False
        x=s.actors[0].x
        for _ in range(10):s.step(.04)
        self.assertEqual(s.actors[0].x,x)

if __name__=='__main__':
    unittest.main()
