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
TRAVEL_RADIUS = BODY_RADIUS + 0.014
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
    pitch: float = 0.0

    @property
    def axle(self):
        offset = WHEEL_RADIUS * math.sin(self.pitch)
        return self.x + offset * math.cos(self.yaw), self.y + offset * math.sin(self.yaw)

    def distance(self, target):
        return math.hypot(target[0] - self.x, target[1] - self.y)


class Odometry:
    def __init__(self, pose):
        self.pose = pose
        self.previous = None

    def update(self, left, right, yaw, pitch=0.0):
        if not all(math.isfinite(v) for v in (left, right, yaw, pitch)):
            raise ValueError('Invalid encoder or IMU reading')
        if self.previous is not None:
            distance = WHEEL_RADIUS * (((left - self.previous[0]) +
                                        (right - self.previous[1])) / 2 +
                                       pitch - self.previous[2])
            heading = self.pose.yaw + wrap(yaw - self.pose.yaw) / 2
            # Wheel centers are 20mm above the Robot origin. With a pitched
            # chassis, that origin orbits the axle when turning; integrate the
            # axle, then transform back to the origin used by the sensor map.
            old_offset = WHEEL_RADIUS * math.sin(self.previous[2])
            new_offset = WHEEL_RADIUS * math.sin(pitch)
            self.pose.x += (distance * math.cos(heading) +
                            old_offset * math.cos(self.pose.yaw) - new_offset * math.cos(yaw))
            self.pose.y += (distance * math.sin(heading) +
                            old_offset * math.sin(self.pose.yaw) - new_offset * math.sin(yaw))
        self.previous = left, right, pitch
        self.pose.yaw = wrap(yaw)
        self.pose.pitch = pitch


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


def ray_height(distance, relative_angle, pitch):
    """Approximate beam height while the base settles between sensor rings."""
    mount_pitch = math.atan2(0.6, 0.8) if pitch > 0.3 else 0.0
    return 0.04 - math.sin(pitch-mount_pitch)*math.cos(relative_angle)*distance


class OccupancyGrid:
    """Incremental ray map and footprint-inflated path search."""
    resolution = 0.025
    width = 98
    height = 54

    def __init__(self):
        self.evidence = {}
        self.visits = {}
        self.covered = set()
        self.goal = None
        self.escaping = False
        self.exit_samples = []

    def cell(self, x, y):
        return math.floor(x / self.resolution), math.floor(y / self.resolution)

    def point(self, cell):
        return tuple((v + 0.5) * self.resolution for v in cell)

    def inside(self, cell):
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def update(self, pose, ranges):
        here = self.cell(*pose.axle)
        if here not in self.visits:
            # Sparse rays leave small unknown slivers even around places we
            # have already inspected from all directions. Do not select those
            # slivers as new exploration goals and shuttle along the same path.
            radius = math.ceil(0.10 / self.resolution)
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if math.hypot(dx, dy) * self.resolution <= 0.10:
                        self.covered.add((here[0] + dx, here[1] + dy))
        self.visits[here] = self.visits.get(here, 0) + 1
        free_cells, hit_cells = set(), set()
        for index, distance in enumerate(ranges):
            angle = pose.yaw + TAU * index / RAY_COUNT
            c, s = math.cos(angle), math.sin(angle)
            length = min(RANGE, distance) + SENSOR_RADIUS
            for step in range(int((length - 0.025) / 0.02)):
                cell = self.cell(pose.x + step * 0.02 * c, pose.y + step * 0.02 * s)
                if self.inside(cell):
                    free_cells.add(cell)
            if distance < RANGE - 0.01:
                cell = self.cell(pose.x + length * c, pose.y + length * s)
                if self.inside(cell):
                    hit_cells.add(cell)
            # Discover the transition from rays that pass through the north wall.
            # No transition X coordinate or obstacle position is supplied.
            if s > 0.15 and pose.y < 1.20 and pose.y + length * s > 1.30:
                x = pose.x + (1.22 - pose.y) * c / s
                height = ray_height((1.22-pose.y)/s, TAU*index/RAY_COUNT, pose.pitch)
                # A tilted ray may pass over the 10cm boundary wall. Only
                # crossings within the wall's vertical span prove an opening.
                if 0.06 < x < 2.38 and 0.015 < height < 0.085:
                    self.exit_samples.append(x)
                    self.exit_samples = self.exit_samples[-100:]
        # A grazing ray may cross the empty part of a voxel hit by another
        # ray. Aggregate the scan first: a measured surface wins over free
        # space, and each voxel gets exactly one vote per complete scan.
        for cell in free_cells - hit_cells:
            self.evidence[cell] = max(-8, self.evidence.get(cell, 0) - 1)
        for cell in hit_cells:
            self.evidence[cell] = min(12, self.evidence.get(cell, 0) + 4)

    def blocked(self):
        occupied = {cell for cell, value in self.evidence.items() if value >= 2}
        blocked = set()
        padding = math.ceil(TRAVEL_RADIUS / self.resolution) + 1
        for x, y in occupied:
            for dx in range(-padding, padding + 1):
                for dy in range(-padding, padding + 1):
                    if math.hypot(max(0, abs(dx) - 0.5), max(0, abs(dy) - 0.5)) * self.resolution < TRAVEL_RADIUS:
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
        self.goal = None
        self.escaping = False
        start = self.cell(*pose.axle)
        blocked = self.blocked()
        if start in blocked:
            # A newly seen obstacle can put the current cell inside the extra
            # safety margin. Bridge outward to free space rather than spinning
            # in a cell whose inflated neighbours all forbid an ordinary path.
            origin = pose.axle
            nearby = [self.point(c) for c, v in self.evidence.items()
                      if v >= 2 and math.dist(self.point(c), origin) < 0.35]
            def clearance(point):
                return min((math.hypot(max(0, abs(point[0]-p[0])-self.resolution/2),
                                       max(0, abs(point[1]-p[1])-self.resolution/2))
                            for p in nearby), default=1.0)
            minimum = clearance(origin) - 0.001
            candidates = []
            for dx in range(-4, 5):
                for dy in range(-4, 5):
                    cell = start[0]+dx, start[1]+dy
                    if not self.inside(cell) or cell in blocked or self.evidence.get(cell, 0) >= 0:
                        continue
                    point = self.point(cell)
                    steps = max(1, math.ceil(math.dist(origin, point)/0.005))
                    if all(clearance((origin[0]+(point[0]-origin[0])*i/steps,
                                      origin[1]+(point[1]-origin[1])*i/steps)) >= minimum
                           for i in range(steps+1)):
                        candidates.append(point)
            if candidates:
                self.escaping = True
                return [min(candidates, key=lambda p: math.dist(origin, p))]
        distances, parents = self.search(start, blocked - {start})
        if self.exit_samples:
            sx = sorted(self.exit_samples)
            goal = self.cell(sx[len(sx) // 2], 1.27)
            if goal not in distances:
                goal = min(distances, key=lambda c: math.dist(c, goal))
        else:
            frontiers = [c for c in distances if c not in self.covered
                         and self.evidence.get(c, 0) < 0
                         and self.point(c)[1] < 1.17
                         and any(self.evidence.get((c[0] + dx, c[1] + dy), 0) == 0
                                 for dx, dy in DIRECTIONS)]
            if not frontiers:
                return []
            goal = min(frontiers, key=lambda c: distances[c] - 2 * c[1] + 0.03 * self.visits.get(c, 0))
        self.goal = goal
        path = [goal]
        while path[-1] != start:
            path.append(parents[path[-1]])
        path.reverse()
        if len(path) < 2:
            return []
        waypoints, index = [], 0
        while index < len(path) - 1:
            end = index + 1
            direction = (path[end][0] - path[index][0], path[end][1] - path[index][1])
            while end + 1 < len(path) and end - index < 6:
                a, b = path[end], path[end + 1]
                if (b[0] - a[0], b[1] - a[1]) != direction:
                    break
                end += 1
            waypoints.append(self.point(path[end]))
            index = end
        return waypoints


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
    # Steering and arrival refer to the drive axle. The chassis origin sits
    # behind it when tilted; steering that rear point chases its own rotation
    # around tiny centering targets and never converges.
    x, y = pose.axle
    distance = math.hypot(target[0] - x, target[1] - y)
    if distance < tolerance:
        return 0.0, 0.0, True
    error = wrap(math.atan2(target[1] - y, target[0] - x) - pose.yaw)
    turn = clamp(error * 3.0, -2.2, 2.2)
    speed = min(0.10, 1.5 * distance) if abs(error) < 0.22 else 0.0
    for index, reading in enumerate(ranges):
        angle = TAU * index / RAY_COUNT
        x = (reading + SENSOR_RADIUS) * math.cos(angle)
        y = (reading + SENSOR_RADIUS) * math.sin(angle)
        if x > 0 and abs(y) < TRAVEL_RADIUS:
            # Translation sweeps a circle along +X. A point near its side
            # meets the circular cap later than a point directly ahead.
            # A rectangular cap incorrectly stops at safe diagonal corners.
            cap = math.sqrt(max(0, TRAVEL_RADIUS ** 2 - y ** 2))
            speed = min(speed, max(0, (x - cap) * 2))
    left = (speed - turn * AXLE / 2) / WHEEL_RADIUS
    right = (speed + turn * AXLE / 2) / WHEEL_RADIUS
    factor = max(1, abs(left) / max_speed, abs(right) / max_speed)
    return left / factor, right / factor, False
