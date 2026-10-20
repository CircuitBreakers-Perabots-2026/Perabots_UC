"""PeraBots Phase 1: autonomous mapping, exploration and majority indication."""
import json
import math
from navigation import (PITCH, RAY_COUNT, Maze, OccupancyGrid, Odometry, Pose,
                        cardinal_ranges, clamp, drive_to, floor_color, ray_height, wrap)


class Mission:
    def __init__(self):
        self.pose = Pose(0, 0, 0)
        self.odometry = Odometry(self.pose)
        self.field = OccupancyGrid()
        self.maze = Maze()
        self.phase = 'initializing'
        self.target = None
        self.field_route = []
        self.cell = (0, 0)
        self.origin = None
        self.last_plan = -1
        self.black_entry = None
        self.samples = []
        self.last_progress = 0
        self.progress_pose = (0, 0)
        self.recover_until = 0
        self.error = None
        self.color = 'unknown'

    def start(self, ranges, yaw, color, now):
        distances = cardinal_ranges(ranges, yaw)
        # Localize within the designated start zone using its nearby boundaries.
        corridor = distances[0] < 0.20 and distances[2] < 0.20
        if color in ('black', 'blue', 'green') or (color == 'red' and corridor):
            self.pose.x = self.pose.y = 0
            self.phase = 'chamber' if color == 'red' else 'maze_entry'
            if self.phase == 'maze_entry':
                self.target = (0, 0.119 - distances[3]) if distances[3] < 0.24 else (0, 0)
        else:
            self.pose.x = 2.44 - 0.006 - distances[0] if distances[0] < 0.3 else 2.315
            self.pose.y = 0.006 + distances[3] if distances[3] < 0.3 else 0.125
            self.phase = 'field'
        self.pose.yaw = yaw
        self.last_progress = now
        self.progress_pose = self.pose.x, self.pose.y

    def wall_is_level(self, distance, direction):
        height = ray_height(distance, direction*math.pi/2-self.pose.yaw, self.pose.pitch)
        return 0.01 < height < 0.09

    def enter_maze(self, distances):
        self.origin = self.pose.x, self.pose.y
        self.cell = (0, 0)
        # Any north-facing wall lies on a maze grid line. Its range modulo
        # the pitch removes floor-camera offset and slip at the red/black edge.
        # The nearest west boundary supplies the entrance's lateral alignment.
        for axis, direction, sign in ((0, 2, -1), (1, 1, 1)):
            distance = distances[direction]
            if (0.06 < distance < (2.3 if axis == 0 else 1.05) and
                    self.wall_is_level(distance, direction)):
                expected = 0.119 + round((distance - 0.119) / PITCH) * PITCH
                offset = sign * (expected - distance)
                if abs(offset) < 0.08:
                    if axis == 0:
                        self.pose.x += offset
                    else:
                        self.pose.y += offset
        self.phase = 'maze_centering'
        self.target = self.origin
        self.samples = []

    def center(self, cell):
        return self.origin[0] + cell[0] * PITCH, self.origin[1] + cell[1] * PITCH

    def align_center(self, distances):
        nominal = self.center(self.cell)
        # Every maze wall lies on a grid line, including walls several cells
        # away. Correct corridor slip before it accumulates at the first turn.
        for axis, positive, negative in ((0, 0, 2), (1, 1, 3)):
            estimates = []
            for direction, sign, cells in ((positive, 1, (8 if axis == 0 else 3)-self.cell[axis]),
                                            (negative, -1, self.cell[axis])):
                distance = distances[direction]
                count = round((distance-0.119)/PITCH)
                expected = 0.119 + count*PITCH
                if (distance > 0.05 and 0 <= count <= cells and self.wall_is_level(distance, direction) and
                        abs(distance-expected) < 0.08):
                    estimates.append(nominal[axis] + sign*(expected-distance))
            if estimates:
                value = sum(estimates) / len(estimates)
                current = self.pose.x if axis == 0 else self.pose.y
                value = current + clamp(value - current, -0.025, 0.025)
                if axis == 0:
                    self.pose.x = value
                else:
                    self.pose.y = value

    def step(self, now, ranges, encoders, yaw, color, max_speed=6.0, ahead_color='unknown', pitch=0.0):
        if len(ranges) != RAY_COUNT or not math.isfinite(yaw) or any(not math.isfinite(r) or r < 0 for r in ranges):
            raise ValueError('Invalid range/heading reading')
        self.color = color
        self.wall_ranges = cardinal_ranges(ranges, yaw)
        if self.phase == 'initializing':
            self.start(ranges, yaw, color, now)
        self.odometry.update(*encoders, yaw, pitch)
        if self.phase in ('finished', 'tie', 'failed'):
            return 0.0, 0.0
        if math.dist((self.pose.x, self.pose.y), self.progress_pose) > 0.025:
            self.progress_pose = self.pose.x, self.pose.y
            self.last_progress = now

        if self.phase == 'field':
            self.field.update(self.pose, ranges)
            if color == 'red' and self.pose.y > 0.5:
                self.phase, self.target = 'chamber', None
                return 0.0, 0.0
            # Finish the selected segment before choosing its successor. Planning
            # every few frames changes the turn waypoint while we approach a
            # cell boundary and can alternate left/right forever at a corner.
            if self.target is None:
                if not self.field_route:
                    self.field_route = self.field.route(self.pose)
                self.target = self.field_route.pop(0) if self.field_route else None
                self.last_plan = now
            if self.target is None:
                return -1.8, 1.8
            if now - self.last_progress > 5:
                self.target = None
                self.field_route.clear()
                self.last_plan = -1
                self.recover_until = now + 0.6
                self.last_progress = now
            if now < self.recover_until:
                rear = ranges[RAY_COUNT // 2] + 0.045
                return (-1.5, -1.5) if rear > 0.10 else (-1.8, 1.8)
            if self.target is None:
                return 0.0, 0.0
            left, right, arrived = drive_to(self.pose, self.target, ranges, max_speed,
                                           0.003 if self.field.escaping else 0.012)
            if arrived:
                self.target = None
            return left, right

        if self.phase == 'chamber':
            # Follow the center north; anchor the maze at the red/black edge.
            error = wrap(math.pi / 2 - yaw)
            if abs(error) > 0.035:
                turn = clamp(error * 3, -1.8, 1.8)
                return -turn, turn
            distances = cardinal_ranges(ranges, yaw)
            correction = 0
            if distances[0] < 0.20 and distances[2] < 0.20:
                correction = clamp((distances[2] - distances[0]) * 12, -0.6, 0.6)
            if color == 'black' and self.black_entry is None:
                self.black_entry = self.pose.y
            if self.black_entry is not None and self.pose.y - self.black_entry >= 0.132:
                self.enter_maze(cardinal_ranges(ranges, yaw))
                return 0.0, 0.0
            return 2.8 - correction, 2.8 + correction

        if self.phase == 'maze_entry':
            left, right, arrived = drive_to(self.pose, self.target, ranges, max_speed)
            if arrived:
                distances = cardinal_ranges(ranges, yaw)
                if distances[0] < 0.20 and distances[2] < 0.20:
                    self.target = (self.pose.x + (distances[0] - distances[2]) / 2, self.pose.y)
                    if math.dist(self.pose.axle, self.target) > 0.012:
                        return left, right
                self.enter_maze(cardinal_ranges(ranges, yaw))
            return left, right

        if self.phase == 'maze_centering':
            left, right, arrived = drive_to(self.pose, self.target, ranges, max_speed, 0.008)
            if arrived:
                self.phase, self.target = 'maze', None
                self.last_progress = now
            elif now - self.last_progress > 10:
                raise RuntimeError('Cannot center in the maze entrance')
            return left, right

        if self.phase == 'maze':
            if (self.target is not None and ahead_color == 'red'
                    and self.destination not in self.maze.visited
                    and self.maze.end != self.destination
                    and abs(wrap(math.atan2(self.target[1] - self.pose.y,
                                           self.target[0] - self.pose.x) - yaw)) < 0.12):
                # Discover the terminal red tile before the body enters it, and
                # return to this cell until the remaining branches are explored.
                self.maze.end = self.destination
                self.destination = self.cell
                self.target = self.center(self.cell)
            if self.target is not None:
                left, right, arrived = drive_to(self.pose, self.target, ranges, max_speed, 0.008)
                if not arrived:
                    if now - self.last_progress > 10:
                        raise RuntimeError('Maze motion blocked: cannot safely reach next cell')
                    return left, right
                self.cell = self.destination
                self.target = None
                self.samples = []
            angle = round(yaw / (math.pi / 2)) * math.pi / 2
            error = wrap(angle - yaw)
            if abs(error) > 0.025:
                turn = clamp(error * 4, -1.5, 1.5)
                return -turn, turn
            distances = cardinal_ranges(ranges, yaw)
            if not all(self.wall_is_level(distance, direction) for direction, distance in enumerate(distances)):
                # A forward ray can hit the floor as the base tips. It must
                # not become a short wall or a false grid-position correction.
                self.samples = []
                return 0.0, 0.0
            self.samples.append(color)
            if len(self.samples) < 3:
                return 0.0, 0.0
            observed = max(set(self.samples), key=self.samples.count)
            self.align_center(distances)
            self.maze.observe(self.cell, distances, observed)
            route = self.maze.next_route(self.cell)
            if not route:
                if not self.maze.explored or self.maze.end != self.cell:
                    raise RuntimeError('Maze exploration incomplete')
                self.phase = 'finished' if self.maze.majority else 'tie'
                return 0.0, 0.0
            self.destination = route[0]
            self.target = self.center(self.destination)
            self.samples = []
            self.last_progress = now
            return 0.0, 0.0
        raise RuntimeError('Unknown mission phase: ' + self.phase)

    def telemetry(self):
        return {'phase': self.phase, 'floor': self.color, 'pose': [self.pose.x, self.pose.y, self.pose.yaw],
                'target': self.target,
                'pitch': self.pose.pitch,
                'walls': getattr(self, 'wall_ranges', []),
                'field_goal': self.field.goal if self.phase == 'field' else None,
                'visited': len(self.maze.visited), 'counts': self.maze.counts,
                'majority': self.maze.majority if self.phase == 'finished' else None,
                'error': self.error}


def main():
    from controller import Robot
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    motors = [robot.getDevice(name + '_wheel_motor') for name in ('left', 'right')]
    for motor in motors:
        motor.setPosition(float('inf'))
        motor.setVelocity(0)
    leds = {color: robot.getDevice('led_' + color) for color in ('blue', 'green')}
    for led in leds.values():
        led.set(0)
    encoders = [robot.getDevice(name + '_wheel_sensor') for name in ('left', 'right')]
    sensors = [robot.getDevice('range_%02d' % i) for i in range(RAY_COUNT)]
    startup_sensors = [robot.getDevice('startup_%02d' % i) for i in range(RAY_COUNT)]
    imu = robot.getDevice('heading')
    camera = robot.getDevice('floor_camera')
    ahead_camera = robot.getDevice('ahead_camera')
    for device in encoders + sensors + startup_sensors + [imu, camera, ahead_camera]:
        device.enable(timestep)
    maximum = min(6.0, *(motor.getMaxVelocity() for motor in motors))
    mission = Mission()
    last_report = None
    while robot.step(timestep) != -1:
        if robot.getTime() < 1.0:
            continue  # Let the locked base settle before localizing or mapping.
        orientation = imu.getRollPitchYaw()
        if 0.08 < orientation[1] < 0.56:
            continue  # Discard transient floor hits while the body tips.
        active_sensors = sensors if orientation[1] > 0.3 else startup_sensors
        try:
            left, right = mission.step(robot.getTime(), [s.getValue() for s in active_sensors],
                                       [s.getValue() for s in encoders], orientation[2],
                                       floor_color(camera.getImage(), camera.getWidth(), camera.getHeight()), maximum,
                                       floor_color(ahead_camera.getImage(), ahead_camera.getWidth(), ahead_camera.getHeight()),
                                       orientation[1])
        except (ValueError, RuntimeError) as error:
            mission.phase, mission.error = 'failed', str(error)
            left = right = 0.0
        motors[0].setVelocity(left)
        motors[1].setVelocity(right)
        for color, led in leds.items():
            led.set(int(mission.phase == 'finished' and mission.maze.majority == color))
        telemetry = mission.telemetry()
        telemetry['leds'] = {color: led.get() for color, led in leds.items()}
        telemetry['wheels'] = [motor.getVelocity() for motor in motors]
        robot.setCustomData(json.dumps(telemetry))
        report = (mission.phase, len(mission.maze.visited))
        if report != last_report:
            print(json.dumps(telemetry), flush=True)
            last_report = report


if __name__ == '__main__':
    main()
