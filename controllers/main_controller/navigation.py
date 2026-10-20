"""Sensor-only navigation. Distances in metres, headings CCW from +X.

No Webots dependency or arena-file access. Prior information is limited to
published dimensions and the designated start/entrance conventions.
"""
from collections import deque
from dataclasses import dataclass
import heapq
import math

TAU = 2 * math.pi
RAY_COUNT = 48
SENSOR_RADIUS = 0.045
RANGE = 3.0
WHEEL_RADIUS = 0.02
AXLE = 0.064
BODY_RADIUS = 0.068
PITCH = 0.25
DIRECTIONS = ((1, 0), (0, 1), (-1, 0), (0, -1))


def wrap(angle):
    return (angle + math.pi) % TAU - math.pi


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


@dataclass
class Pose:
    x: float
    y: float
    yaw: float

    def distance(self, target):
        return math.hypot(target[0] - self.x, target[1] - self.y)


class Odometry:
    def __init__(self, pose):
        self.pose = pose
        self.previous = None

    def update(self, left, right, yaw):
        if not all(math.isfinite(v) for v in (left, right, yaw)):
            raise ValueError('Invalid encoder or IMU reading')
        if self.previous is not None:
            distance = WHEEL_RADIUS * ((left - self.previous[0]) + (right - self.previous[1])) / 2
            heading = self.pose.yaw + wrap(yaw - self.pose.yaw) / 2
            self.pose.x += distance * math.cos(heading)
            self.pose.y += distance * math.sin(heading)
        self.previous = left, right
        self.pose.yaw = wrap(yaw)


def classify_rgb(red, green, blue):
    """Chromatic ratios tolerate lighting, dark tiles and white highlights."""
    if max(red, green, blue) < 12:
        return 'black'
    if red > 35 and red > 1.65 * max(green, blue):
        return 'red'
    if blue > 25 and blue > 1.55 * max(red, green):
        return 'blue'
    if green > 25 and green > 1.55 * max(red, blue):
        return 'green'
    if min(red, green, blue) > 65 and max(red, green, blue) < 1.5 * min(red, green, blue):
        return 'white'
    if max(red, green, blue) < 85 and max(red, green, blue) - min(red, green, blue) < 25:
        return 'black'
    return 'unknown'


def floor_color(image, width, height):
    """Vote over the central image patch; Webots images contain BGRA bytes."""
    if not image:
        return 'unknown'
    votes = {}
    for y in range(height // 3, 2 * height // 3):
        for x in range(width // 3, 2 * width // 3):
            offset = 4 * (y * width + x)
            blue, green, red = image[offset:offset + 3]
            color = classify_rgb(red, green, blue)
            votes[color] = votes.get(color, 0) + 1
    winner = max(votes, key=votes.get)
    return winner if votes[winner] >= 0.65 * sum(votes.values()) else 'unknown'


def cardinal_ranges(ranges, yaw):
    return [ranges[round(wrap(direction * math.pi / 2 - yaw) / TAU * RAY_COUNT) % RAY_COUNT]
            + SENSOR_RADIUS for direction in range(4)]


class OccupancyGrid:
    """Incremental ray map and footprint-inflated path search."""
    resolution = 0.025
    width = 98
    height = 54

    def __init__(self):
        self.evidence = {}
        self.visits = {}
        self.exit_samples = []

    def cell(self, x, y):
        return math.floor(x / self.resolution), math.floor(y / self.resolution)

    def point(self, cell):
        return tuple((v + 0.5) * self.resolution for v in cell)

    def inside(self, cell):
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def update(self, pose, ranges):
        here = self.cell(pose.x, pose.y)
        self.visits[here] = self.visits.get(here, 0) + 1
        for index, distance in enumerate(ranges):
            angle = pose.yaw + TAU * index / RAY_COUNT
            c, s = math.cos(angle), math.sin(angle)
            length = min(RANGE, distance) + SENSOR_RADIUS
            free = set()
            for step in range(int((length - 0.025) / 0.02)):
                cell = self.cell(pose.x + step * 0.02 * c, pose.y + step * 0.02 * s)
                if self.inside(cell):
                    free.add(cell)
            for cell in free:
                self.evidence[cell] = max(-8, self.evidence.get(cell, 0) - 1)
            if distance < RANGE - 0.01:
                cell = self.cell(pose.x + length * c, pose.y + length * s)
                if self.inside(cell):
                    self.evidence[cell] = min(12, self.evidence.get(cell, 0) + 4)
            # Discover the transition from rays that pass through the north wall.
            # No transition X coordinate or obstacle position is supplied.
            if s > 0.15 and pose.y < 1.20 and pose.y + length * s > 1.30:
                x = pose.x + (1.22 - pose.y) * c / s
                if 0.06 < x < 2.38:
                    self.exit_samples.append(x)
                    self.exit_samples = self.exit_samples[-100:]

    def blocked(self):
        occupied = {cell for cell, value in self.evidence.items() if value >= 2}
        blocked = set()
        padding = math.ceil(BODY_RADIUS / self.resolution) + 1
        for x, y in occupied:
            for dx in range(-padding, padding + 1):
                for dy in range(-padding, padding + 1):
                    if math.hypot(max(0, abs(dx) - 0.5), max(0, abs(dy) - 0.5)) * self.resolution < BODY_RADIUS:
                        blocked.add((x + dx, y + dy))
        for x in range(self.width):
            for y in range(self.height):
                px, py = self.point((x, y))
                if px < 0.06 or px > 2.38 or py < 0.06:
                    blocked.add((x, y))
        return blocked

    def search(self, start, blocked):
        distances, parents = {start: 0.0}, {}
        queue = [(0.0, start)]
        while queue:
            cost, cell = heapq.heappop(queue)
            if cost != distances[cell]:
                continue
            for dx, dy in DIRECTIONS:
                nxt = cell[0] + dx, cell[1] + dy
                if not self.inside(nxt) or nxt in blocked:
                    continue
                extra = 1.0 if self.evidence.get(nxt, 0) < 0 else 3.0
                new_cost = cost + extra
                if new_cost < distances.get(nxt, math.inf):
                    distances[nxt], parents[nxt] = new_cost, cell
                    heapq.heappush(queue, (new_cost, nxt))
        return distances, parents

    def route(self, pose):
        start = self.cell(pose.x, pose.y)
        distances, parents = self.search(start, self.blocked() - {start})
        if self.exit_samples:
            sx = sorted(self.exit_samples)
            goal = self.cell(sx[len(sx) // 2], 1.27)
            if goal not in distances:
                goal = min(distances, key=lambda c: math.dist(c, goal))
        else:
            frontiers = [c for c in distances if self.evidence.get(c, 0) < 0
                         and self.point(c)[1] < 1.17
                         and any(self.evidence.get((c[0] + dx, c[1] + dy), 0) == 0
                                 for dx, dy in DIRECTIONS)]
            if not frontiers:
                return []
            goal = min(frontiers, key=lambda c: distances[c] - 2 * c[1] + 0.03 * self.visits.get(c, 0))
        path = [goal]
        while path[-1] != start:
            path.append(parents[path[-1]])
        path.reverse()
        if len(path) < 2:
            return []
        end = 1
        while end + 1 < min(len(path), 7):
            a, b = path[end], path[end + 1]
            if (b[0] - a[0], b[1] - a[1]) != (path[1][0] - start[0], path[1][1] - start[1]):
                break
            end += 1
        return [self.point(path[end])]


class Maze:
    """Learned undirected graph with shortest routes to unvisited cells."""
    def __init__(self):
        self.edges = {}
        self.visited = set()
        self.tiles = {}
        self.end = None

    @staticmethod
    def inside(cell):
        return 0 <= cell[0] < 9 and 0 <= cell[1] < 4

    def observe(self, cell, distances, color):
        self.visited.add(cell)
        self.edges.setdefault(cell, set())
        for direction, (dx, dy) in enumerate(DIRECTIONS):
            nxt = cell[0] + dx, cell[1] + dy
            if self.inside(nxt):
                if distances[direction] > 0.19:
                    self.edges[cell].add(nxt)
                    self.edges.setdefault(nxt, set()).add(cell)
                else:
                    self.edges[cell].discard(nxt)
                    self.edges.get(nxt, set()).discard(cell)
        if color in ('blue', 'green'):
            self.tiles[cell] = color
        elif color == 'red':
            self.end = cell

    def paths(self, start, defer_end=False):
        paths, queue = {start: []}, deque([start])
        while queue:
            cell = queue.popleft()
            for nxt in sorted(self.edges.get(cell, ())):
                if defer_end and nxt == self.end and nxt != start:
                    continue
                if nxt not in paths:
                    paths[nxt] = paths[cell] + [nxt]
                    queue.append(nxt)
        return paths

    def next_route(self, start):
        paths = self.paths(start, defer_end=self.end is not None)
        pending = [cell for cell in paths if cell not in self.visited]
        if not pending:
            paths = self.paths(start)
            pending = [cell for cell in paths if cell not in self.visited]
        if pending:
            return paths[min(pending, key=lambda c: (len(paths[c]), c == self.end, c))]
        if self.end is None:
            raise RuntimeError('All reachable maze cells explored but no red end cell found')
        if self.end not in paths:
            raise RuntimeError('End cell is unreachable')
        return paths[self.end]

    @property
    def explored(self):
        return all(cell in self.visited for cell in self.edges)

    @property
    def counts(self):
        return {color: list(self.tiles.values()).count(color) for color in ('blue', 'green')}

    @property
    def majority(self):
        counts = self.counts
        if counts['blue'] == counts['green']:
            return None  # Rulebook defines no tie indication; never guess.
        return max(counts, key=counts.get)


def drive_to(pose, target, ranges, max_speed=6.0, tolerance=0.012):
    """Turn in place, then track a waypoint with a swept-footprint guard."""
    distance = pose.distance(target)
    if distance < tolerance:
        return 0.0, 0.0, True
    error = wrap(math.atan2(target[1] - pose.y, target[0] - pose.x) - pose.yaw)
    turn = clamp(error * 3.0, -2.2, 2.2)
    speed = min(0.10, 1.5 * distance) if abs(error) < 0.22 else 0.0
    for index, reading in enumerate(ranges):
        angle = TAU * index / RAY_COUNT
        x = (reading + SENSOR_RADIUS) * math.cos(angle)
        y = (reading + SENSOR_RADIUS) * math.sin(angle)
        if x > 0 and abs(y) < BODY_RADIUS:
            speed = min(speed, max(0, (x - BODY_RADIUS - 0.014) * 2))
    left = (speed - turn * AXLE / 2) / WHEEL_RADIUS
    right = (speed + turn * AXLE / 2) / WHEEL_RADIUS
    factor = max(1, abs(left) / max_speed, abs(right) / max_speed)
    return left / factor, right / factor, False
