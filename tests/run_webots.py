"""Run isolated Webots integration fixtures; never alter competition assets.

Usage: python3 tests/run_webots.py --scenario maze --output output/maze.json
The test supervisor exists only in a temporary project, never in the submission.
"""
import argparse
import hashlib
import json
import random
import re
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import textwrap

ROOT = Path(__file__).resolve().parents[1]
WEBOTS = Path('/Applications/Webots.app/Contents/MacOS/webots')


def solid(name, x, y, sx, sy, color=None):
    appearance = ' '.join(str(v) for v in (color or (1, 1, 1)))
    height = 0.001 if color else 0.1
    z = 0.001 if color else 0.05
    collision = '' if color else f'boundingObject Box {{ size {sx} {sy} {height} }}'
    return f'''Solid {{
      translation {x} {y} {z}
      name "{name}"
      children [ Shape {{
        appearance PBRAppearance {{ baseColor {appearance} roughness 1 metalness 0 }}
        geometry Box {{ size {sx} {sy} {height} }}
      }} ]
      {collision}
    }}'''


def generated_maze(project, seed=42, blue_majority=False):
    """A different connected maze with a red terminal near the entrance."""
    rng = random.Random(seed)
    cells = [(x, y) for x in range(9) for y in range(4)]
    parent = {cell: cell for cell in cells}
    def root(cell):
        while parent[cell] != cell:
            cell = parent[cell]
        return cell
    pairs = [(c, (c[0]+dx, c[1]+dy)) for c in cells for dx, dy in ((1, 0), (0, 1))
             if (c[0]+dx, c[1]+dy) in parent]
    rng.shuffle(pairs)
    edges = {cell: set() for cell in cells}
    for a, b in pairs:
        if root(a) != root(b):
            parent[root(a)] = root(b)
            edges[a].add(b)
            edges[b].add(a)
    leaves = [cell for cell in cells if len(edges[cell]) == 1 and cell != (0, 0)]
    end = min(leaves, key=lambda c: (sum(c), c))
    leaves.remove(end)
    rng.shuffle(leaves)
    tiles = {cell: ('blue' if i < 2 else 'green') for i, cell in enumerate(leaves[:5])}
    if blue_majority:
        tiles = {cell: ('green' if color == 'blue' else 'blue') for cell, color in tiles.items()}
    colors = {'blue': (0, 0, 0.5), 'green': (0, 0.6, 0), 'red': (1, 0, 0)}
    tiles[end] = 'red'
    path = project / 'protos/Arena.proto'
    original = path.read_text()
    # Balanced extraction preserves all original floor and boundary geometry.
    chunks = []
    cursor = 0
    while True:
        start = original.find('Solid {', cursor)
        if start == -1:
            break
        depth, stop = 1, start + len('Solid {')
        while depth:
            if original[stop] == '{':
                depth += 1
            elif original[stop] == '}':
                depth -= 1
            stop += 1
        block = original[start:stop]
        if '"maze_' not in block and '"end_marker"' not in block:
            chunks.append(block)
        cursor = stop
    for c in cells:
        x, y = 0.22 + c[0]*0.25, 1.455 + c[1]*0.25
        for dx, dy in ((1, 0), (0, 1)):
            nxt = c[0]+dx, c[1]+dy
            if nxt in edges and nxt not in edges[c]:
                chunks.append(solid('test_wall_%s_%s_%s_%s' % (*c, dx, dy),
                                    x + dx*0.125, y + dy*0.125,
                                    0.012 if dx else 0.262, 0.262 if dx else 0.012))
        if c in tiles:
            chunks.append(solid('test_tile_%s_%s' % c, x, y, 0.25, 0.25, colors[tiles[c]]))
    path.write_text('#VRML_SIM R2025a utf8\nPROTO Arena [] { Group { children [\n' +
                    '\n'.join(chunks) + '\n] } }\n')
    return (0.22 + end[0]*0.25, 1.455 + end[1]*0.25), {
        color: list(tiles.values()).count(color) for color in ('blue', 'green')}


def generated_obstacles(project, seed=84):
    rng = random.Random(seed)
    positions = [(x, y) for x in (0.25, 0.60, 0.95, 1.30, 1.65, 2.0)
                 for y in (0.25, 0.60, 0.85)]
    rng.shuffle(positions)
    blocks = [solid('test_obstacle_%s' % i, x, y, 0.1, 0.1) for i, (x, y) in enumerate(positions[:14])]
    (project / 'protos/Obstacles.proto').write_text(
        '#VRML_SIM R2025a utf8\nPROTO Obstacles [] { Group { children [\n' +
        '\n'.join(blocks) + '\n] } }\n')


def obstacle_rectangles(project):
    """Read fixture truth only in the test harness, never in robot runtime."""
    rectangles = []
    for filename in ('Arena.proto', 'Obstacles.proto'):
        source = (project / 'protos' / filename).read_text()
        for match in re.finditer(r'Solid \{', source):
            start, stop, depth = match.start(), match.end(), 1
            while depth:
                if source[stop] == '{':
                    depth += 1
                elif source[stop] == '}':
                    depth -= 1
                stop += 1
            block = source[start:stop]
            size = re.search(r'boundingObject Box \{\s*size ([\d.e+-]+) ([\d.e+-]+) ([\d.e+-]+)', block)
            position = re.search(r'translation ([\d.e+-]+) ([\d.e+-]+) ([\d.e+-]+)', block)
            if size and position:
                sx, sy, sz = map(float, size.groups())
                x, y, _ = map(float, position.groups())
                if sz > 0.05:
                    rectangles.append((x-sx/2, x+sx/2, y-sy/2, y+sy/2))
    return rectangles


def prepare(scenario, output, seed=42, blue_majority=False, official_lighting=False):
    project = Path(tempfile.mkdtemp(prefix='perabots-webots-'))
    for folder in ('controllers', 'protos', 'worlds'):
        shutil.copytree(ROOT / folder, project / folder, ignore=shutil.ignore_patterns('__pycache__'))
    world = (project / 'worlds/Arena.wbt').read_text()
    world = world.replace('TwoWheelRobot {', 'DEF TEAM TwoWheelRobot {')
    expected_end, expected_counts = (2.21, 2.205), {'blue': 2, 'green': 3}
    if scenario.startswith('generated'):
        expected_end, expected_counts = generated_maze(project, seed, blue_majority)
        generated_obstacles(project, seed + 42)
    if scenario in ('maze', 'generated-maze'):
        world = world.replace('translation 2.315 0.125 0.0025', 'translation 0.22 1.455 0.0025')
        world = world.replace('rotation 0 0 1 3.14159265359', 'rotation 0 0 1 1.57079632679')
    if scenario == 'reported-start':
        # The supplied screenshot's original saved pose, not our centered reset.
        world = world.replace('translation 2.315 0.125 0.0025',
                              'translation 2.26866 0.159772 0.00245697')
        world = world.replace('rotation 0 0 1 3.14159265359',
                              'rotation -0.026775583990977556 0.010250747447849066 0.9995889106421213 2.3235643717090495')
    if scenario == 'chamber':
        world = world.replace('translation 2.315 0.125 0.0025', 'translation 0.22 1.275 0.0025')
        world = world.replace('rotation 0 0 1 3.14159265359', 'rotation 0 0 1 1.57079632679')
    if not official_lighting:
        # Fixture lighting avoids network dependencies on Webots asset downloads.
        world = re.sub(r'EXTERNPROTO "https://[^\n]*TexturedBackground[^\n]*\n', '', world)
        world = re.sub(r'TexturedBackground(?:Light)? \{[^}]*\}', '', world)
        world += '\nBackground { skyColor [ 0.6 0.6 0.6 ] }\n'
    world += '\nRobot { supervisor TRUE controller "test_supervisor" }\n'
    (project / 'worlds/Arena.wbt').write_text(world)
    supervisor = project / 'controllers/test_supervisor'
    supervisor.mkdir()
    source = '''
from controller import Supervisor
import json
import math
from pathlib import Path
robot = Supervisor()
team = robot.getFromDef('TEAM')
field = team.getBaseNodeField('customData')
previous = team.getPosition()
distance = 0
last_report = -10
last_state = None
last_motion = 0
anchor = previous
contacts = 0
in_contact = False
end_entry = None
first_end_entry = None
finish_seen = None
finish_position = None
early_led = False
outside_since = None
result = {}
# Chassis corners and circumspheres for the caster, wheels and LED lenses.
# Transform the actual tilted body, rather than assuming an upright footprint.
body = [(x, y, z, 0) for x in (-0.04, 0.04) for y in (-0.025, 0.025)
        for z in (0.025, 0.055)]
body += [(0, y, 0.02, 0.021) for y in (-0.032, 0.032)]
body += [(-0.03, 0, 0.008, 0.008)]
body += [(0.02, y, 0.06, 0.005) for y in (-0.012, 0.012)]
while robot.step(32) != -1:
    now = robot.getTime()
    p = team.getPosition()
    distance += math.dist(p[:2], previous[:2])
    previous = p
    if math.dist(p[:2], anchor[:2]) > 0.025:
        last_motion, anchor = now, p
    # Include low chassis contacts: floor points are ignored only when their
    # XY position is outside every fixture obstacle/wall footprint.
    hit = any(rect[0]-0.0005 <= contact.point[0] <= rect[1]+0.0005 and
              rect[2]-0.0005 <= contact.point[1] <= rect[3]+0.0005
              for contact in team.getContactPoints(True) for rect in RECTANGLES)
    if hit and not in_contact:
        contacts += 1
    in_contact = hit
    try:
        data = json.loads(field.getSFString())
    except (ValueError, TypeError):
        data = {}
    orientation = team.getOrientation()
    inside_end = all(abs(p[axis] + sum(orientation[3*axis+i] * point[i] for i in range(3))
                         - EXPECTED_END[axis]) + point[3] < 0.119
                     for point in body for axis in (0, 1))
    bounds = [(min(p[axis] + sum(orientation[3*axis+i] * point[i] for i in range(3)) - point[3]
                   for point in body),
               max(p[axis] + sum(orientation[3*axis+i] * point[i] for i in range(3)) + point[3]
                   for point in body)) for axis in (0, 1)]
    outside = any(upper < 0 or lower > 2.44 for lower, upper in bounds)
    if outside and outside_since is None:
        outside_since = now
    if not outside:
        outside_since = None
    if inside_end and end_entry is None:
        end_entry = now
    if not inside_end:
        end_entry = None
    state = data.get('phase'), data.get('visited')
    if now - last_report > 10 or state != last_state:
        print('MONITOR', round(now, 2), [round(v, 3) for v in p], data, flush=True)
        last_report = now
        last_state = state
    phase = data.get('phase')
    if inside_end and first_end_entry is None:
        first_end_entry = now
    if phase != 'finished' and any(data.get('leds', {}).values()):
        early_led = True
    reason = None
    if phase in ('failed', 'tie'):
        reason = phase
    elif phase == 'finished':
        if finish_seen is None:
            finish_seen, finish_position = now, p
        if now - finish_seen >= 0.5:
            reason = 'finished'
    elif first_end_entry is not None and now - first_end_entry > 3:
        reason = 'end_deadline_missed'
    elif outside_since is not None and now - outside_since > 2:
        reason = 'outside_track_2s'
    elif now > 650:
        reason = 'timeout'
    elif now - last_motion > 15:
        reason = 'no_progress_15s'
    if reason:
        result = dict(reason=reason, time=now, distance=distance, collisions=contacts,
                      position=p, inside_end=inside_end,
                      led_delay=None if first_end_entry is None or finish_seen is None else finish_seen-first_end_entry,
                      stopped_drift=None if finish_position is None else math.dist(p[:2], finish_position[:2]),
                      early_led=early_led, telemetry=data)
        result.update(CONFIGURATION)
        result['passed'] = (reason == 'finished' and inside_end and
            data.get('counts') == EXPECTED_COUNTS and data.get('majority') == MAJORITY and
            result['led_delay'] is not None and result['led_delay'] <= 3 and
            result['stopped_drift'] < 0.005 and not early_led and contacts == 0 and
            data.get('leds') == {c: int(c == MAJORITY) for c in ('blue', 'green')} and data.get('wheels') == [0, 0])
        Path(OUTPUT).write_text(json.dumps(result, indent=2))
        print('RESULT', json.dumps(result), flush=True)
        robot.simulationQuit(0 if result['passed'] else 1)
        break
'''
    configuration = {'scenario': scenario, 'seed': seed, 'official_lighting': official_lighting,
                     'runtime_sha256': {str(path): hashlib.sha256((project/path).read_bytes()).hexdigest()
                                        for path in (Path('controllers/main_controller/main_controller.py'),
                                                     Path('controllers/main_controller/navigation.py'),
                                                     Path('protos/TwoWheelRobot.proto'))}}
    source = ('CONFIGURATION = ' + repr(configuration) + '\nOUTPUT = ' + repr(str(output)) + '\nEXPECTED_END = ' + repr(expected_end) +
              '\nEXPECTED_COUNTS = ' + repr(expected_counts) + '\nMAJORITY = ' +
              repr(max(expected_counts, key=expected_counts.get)) + '\nRECTANGLES = ' +
              repr(obstacle_rectangles(project)) + '\n' + textwrap.dedent(source))
    (supervisor / 'test_supervisor.py').write_text(source)
    return project


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=('maze', 'chamber', 'continuous', 'generated-maze', 'generated-continuous', 'reported-start'), default='maze')
    parser.add_argument('--output', type=Path, default=ROOT / 'output/webots-result.json')
    parser.add_argument('--official-lighting', action='store_true')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--blue-majority', action='store_true')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.unlink(missing_ok=True)
    project = prepare(args.scenario, output, args.seed, args.blue_majority, args.official_lighting)
    print('Test project:', project, flush=True)
    if args.prepare_only:
        return
    command = ['arch', '-x86_64', str(WEBOTS), '--batch', '--mode=fast', '--minimize', '--no-rendering', '--stdout', '--stderr', str(project / 'worlds/Arena.wbt')]
    try:
        process = subprocess.run(command, timeout=900)
        if not output.exists():
            raise RuntimeError('Webots exited without a test result; code %s' % process.returncode)
        result = json.loads(output.read_text())
        print(json.dumps(result, indent=2))
        sys.exit(0 if result['passed'] else 1)
    except subprocess.TimeoutExpired:
        raise RuntimeError('Webots exceeded the 15-minute wall-clock test limit')


if __name__ == '__main__':
    main()
