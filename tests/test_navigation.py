"""Behavioral tests for changed layouts, counts, motion safety and compliance."""
import hashlib
import math
from pathlib import Path
import random
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'controllers/main_controller'))
from navigation import (DIRECTIONS, RAY_COUNT, SENSOR_RADIUS, Maze, OccupancyGrid,
                        Odometry, Pose, cardinal_ranges, classify_rgb, drive_to,
                        floor_color, wrap)
from main_controller import Mission


class PerceptionTests(unittest.TestCase):
    def test_lighting_and_dark_colors(self):
        for rgb, expected in [((0, 0, 0), 'black'), ((2, 5, 85), 'blue'),
                              ((10, 155, 8), 'green'), ((180, 20, 12), 'red'),
                              ((210, 220, 208), 'white'), ((18, 16, 15), 'black')]:
            self.assertEqual(classify_rgb(*rgb), expected)

    def test_camera_bgra_and_empty_frame(self):
        self.assertEqual(floor_color(bytes([120, 4, 2, 255]) * 24 * 24, 24, 24), 'blue')
        self.assertEqual(floor_color(None, 24, 24), 'unknown')

    def test_rotated_cardinal_sensor_order(self):
        ranges = [float(i) for i in range(RAY_COUNT)]
        self.assertEqual(cardinal_ranges(ranges, math.pi / 2),
                         [ranges[i] + SENSOR_RADIUS for i in (36, 0, 12, 24)])


class MotionTests(unittest.TestCase):
    def test_encoder_distance_and_heading_wrap(self):
        pose = Pose(0, 0, math.pi - 0.01)
        odometry = Odometry(pose)
        odometry.update(0, 0, math.pi - 0.01)
        odometry.update(5, 5, -math.pi + 0.01)
        self.assertAlmostEqual(pose.x, -0.1, places=5)
        self.assertAlmostEqual(pose.y, 0, places=5)
        self.assertAlmostEqual(wrap(2 * math.pi + 0.5), 0.5)

    def test_turn_before_driving_and_speed_limit(self):
        l, r, done = drive_to(Pose(0, 0, 0), (0, 1), [3] * RAY_COUNT, 2)
        self.assertFalse(done)
        self.assertLess(l, 0)
        self.assertGreater(r, 0)
        self.assertLessEqual(max(abs(l), abs(r)), 2)

    def test_obstacle_stops_forward_motion(self):
        rays = [3] * RAY_COUNT
        rays[0] = 0.015
        l, r, done = drive_to(Pose(0, 0, 0), (1, 0), rays)
        self.assertFalse(done)
        self.assertEqual((l, r), (0, 0))

    def test_invalid_sensor_fails_closed(self):
        mission = Mission()
        with self.assertRaises(ValueError):
            mission.step(0, [math.nan] * RAY_COUNT, (0, 0), 0, 'white')

    def test_maze_entry_corrects_floor_edge_offset(self):
        mission = Mission()
        mission.pose = Pose(0, 0.18, math.pi/2)
        mission.odometry.pose = mission.pose
        mission.enter_maze([0.119, 0.899, 0.119, 1.5])
        self.assertEqual(mission.phase, 'maze_centering')
        self.assertAlmostEqual(mission.pose.y, 0.15)
        self.assertEqual(mission.target, (0, 0.18))

    def test_terminal_state_stops(self):
        for phase in ('finished', 'tie', 'failed'):
            mission = Mission()
            mission.phase = phase
            self.assertEqual(mission.step(1, [3] * RAY_COUNT, (0, 0), 0, 'black'), (0, 0))


class MazeTests(unittest.TestCase):
    def run_layout(self, seed, add_loops=False):
        rng = random.Random(seed)
        graph = {(x, y): set() for x in range(9) for y in range(4)}
        visited, stack = {(0, 0)}, [(0, 0)]
        while stack:
            cell = stack[-1]
            candidates = [(cell[0]+dx, cell[1]+dy) for dx, dy in DIRECTIONS
                          if (cell[0]+dx, cell[1]+dy) in graph and (cell[0]+dx, cell[1]+dy) not in visited]
            if candidates:
                nxt = rng.choice(candidates)
                graph[cell].add(nxt)
                graph[nxt].add(cell)
                visited.add(nxt)
                stack.append(nxt)
            else:
                stack.pop()
        if add_loops:
            for cell in graph:
                for dx, dy in DIRECTIONS:
                    nxt = cell[0]+dx, cell[1]+dy
                    if nxt in graph and rng.random() < 0.25:
                        graph[cell].add(nxt)
                        graph[nxt].add(cell)
        dead_ends = [c for c in graph if len(graph[c]) == 1 and c != (0, 0)]
        rng.shuffle(dead_ends)
        colors = {c: ('blue' if i % 3 == 0 else 'green') for i, c in enumerate(dead_ends[:-1])}
        end = dead_ends[-1] if dead_ends else (8, 3)
        colors[end] = 'red'
        maze = Maze()
        cell = (0, 0)
        for _ in range(400):
            readings = [0.4 if (cell[0]+dx, cell[1]+dy) in graph[cell] else 0.119 for dx, dy in DIRECTIONS]
            maze.observe(cell, readings, colors.get(cell, 'black'))
            route = maze.next_route(cell)
            if not route:
                break
            self.assertIn(route[0], graph[cell])
            cell = route[0]
        else:
            self.fail('Exploration did not terminate')
        self.assertEqual(maze.visited, set(graph))
        self.assertEqual(cell, end)
        self.assertTrue(maze.explored)
        self.assertEqual(maze.counts, {color: list(colors.values()).count(color) for color in ('blue', 'green')})
        return maze

    def test_changed_maze_layouts_and_cycles(self):
        for seed in range(100):
            with self.subTest(seed=seed):
                self.run_layout(seed, add_loops=bool(seed % 2))

    def test_unique_count_and_tie(self):
        maze = Maze()
        for _ in range(10):
            maze.observe((0, 0), [0.119]*4, 'blue')
        self.assertEqual(maze.counts['blue'], 1)
        self.assertEqual(maze.majority, 'blue')
        maze.observe((1, 0), [0.119]*4, 'green')
        self.assertIsNone(maze.majority)
        maze.tiles[(2, 0)] = 'green'
        self.assertEqual(maze.majority, 'green')

    def test_defer_known_terminal_until_other_branches_explored(self):
        maze = Maze()
        maze.edges = {(0, 0): {(1, 0), (0, 1)}, (1, 0): {(0, 0)}, (0, 1): {(0, 0)}}
        maze.visited = {(0, 0)}
        maze.end = (0, 1)
        self.assertEqual(maze.next_route((0, 0)), [(1, 0)])
        maze.visited.add((1, 0))
        self.assertEqual(maze.next_route((0, 0)), [(0, 1)])

    def test_missing_goal_explicit_error(self):
        maze = Maze()
        maze.observe((0, 0), [0.119]*4, 'black')
        with self.assertRaisesRegex(RuntimeError, 'no red end'):
            maze.next_route((0, 0))


class MappingTests(unittest.TestCase):
    def test_inflation_and_path_avoidance(self):
        grid = OccupancyGrid()
        obstacle = (30, 15)
        grid.evidence[obstacle] = 8
        blocked = grid.blocked()
        self.assertIn((29, 14), blocked)
        start = (20, 15)
        distances, parents = grid.search(start, blocked)
        current = (40, 15)
        self.assertIn(current, distances)
        while current != start:
            self.assertNotIn(current, blocked)
            current = parents[current]

    def test_transition_discovered_from_rays(self):
        grid = OccupancyGrid()
        ranges = [0.3] * RAY_COUNT
        ranges[12] = 0.8
        grid.update(Pose(1.1, 0.8, 0), ranges)
        self.assertTrue(grid.exit_samples)
        self.assertAlmostEqual(grid.exit_samples[0], 1.1)


class ComplianceTests(unittest.TestCase):
    def test_locked_arena_hashes(self):
        expected = {'Arena.proto': '23bb0016ce182e6b71597df623da742a37c9525f13dede044a327745a4f9e82b',
                    'Obstacles.proto': '9c5aee2f03eddd2698fa6b2a5212e3f21fa500027721b4f310cb9bc8508b5401'}
        for file, digest in expected.items():
            self.assertEqual(hashlib.sha256((ROOT / 'protos' / file).read_bytes()).hexdigest(), digest)

    def test_locked_robot_mechanics_and_leds(self):
        proto = (ROOT / 'protos/TwoWheelRobot.proto').read_text()
        body = proto[proto.index('  Robot {'):]
        self.assertEqual(hashlib.sha256(body.encode()).hexdigest(),
                         '3884238c44cb0f289c58224d2a6457488a8b6f3d7e589cbc9759cb1702778234')

    def test_only_permitted_sensor_nodes(self):
        proto = (ROOT / 'protos/TwoWheelRobot.proto').read_text()
        for prohibited in ('GPS {', 'Emitter {', 'Receiver {', 'Lidar {', 'supervisor TRUE'):
            self.assertNotIn(prohibited, proto)
        self.assertEqual(proto.count('DistanceSensor {'), 2 * RAY_COUNT)
        self.assertIn('supervisor FALSE', proto)


if __name__ == '__main__':
    unittest.main()
