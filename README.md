# PeraBots University Category

Autonomous Webots R2025a solution for the Phase 1 obstacle-navigation and
maze-majority challenge. Requires Python 3.8+ and the Webots Python controller
library supplied with Webots. No third-party Python packages are required.

## Run

1. Open `worlds/Arena.wbt` in Webots R2025a.
2. Set Webots' Python command to a working Python 3 installation if necessary.
3. Reset the simulation, then press Run or Fast.

After pulling controller updates, reset the entire simulation before running again
so pose estimation and the map initialize at the designated start.

The robot starts entirely inside the red starting zone. It automatically handles
continuous runs and independent Section 2 placement in the red entrance chamber
or the first black maze cell. Independent placement may face any cardinal direction;
place it near the entrance center, as an evaluator normally would.

The controller prints state transitions and tile counts in the console. The final
state is `finished`, both wheel commands are zero, and exactly the majority LED
is on. `tie` means equal blue/green counts; the PDF specifies no tie indication,
so both LEDs remain off. `failed` includes a diagnostic and stops the motors.

## How it works

- Two 48-ray **DistanceSensor** rings cover the full perimeter. The supplied
  mechanical base can pitch forward under motion; one ring covers the initial
  upright pose and a passive compensated mount covers the settled pose. These
  are individual permitted DistanceSensor devices, not a Lidar device.
- Wheel encoders and an InertialUnit estimate motion, accounting for chassis
  pitch and the offset between the axle and chassis origin. Steering and arrival
  checks use the drive axle. The controller initializes
  within the designated southeast starting zone using measured boundary ranges.
- Section 1 uses a ray-updated occupancy grid, footprint inflation, and path
  search. Scan hits take priority over free-space rays, and each turn waypoint
  stays fixed until arrival. Complete detours are retained across their
  intermediate waypoints. The robot can move out of a newly inflated safety
  margin without moving closer to detected surfaces. Previously inspected neighborhoods are excluded
  from new exploration goals. It discovers the north-boundary opening from sensor rays and recognizes
  the red transition tile. It has no stored obstacle positions or route.
- The red/black floor edge anchors a local 9-by-4 maze grid. Grid-aligned wall
  readings, including distant walls, correct odometry. Floor hits and rays that
  pass over walls while the chassis tips are rejected; the controller learns cell connections and chooses shortest
  routes to unexplored cells. Cell colors are sampled while stopped and counted
  once per cell, including when the robot revisits a dead end.
- A camera looking ahead recognizes the red terminal before entering it and
  defers that branch while other branches remain. After exploration, the robot
  returns to the terminal, stops inside the clear cell, and indicates the majority.

The fixed frame convention is the supplied package's southeast start,
north-boundary transition, southwest maze entrance, and aligned grid. Obstacle
coordinates, internal walls, colored cells, and the terminal cell are learned.
The entrance conventions and field dimensions are configuration assumptions;
this is not a localization system for arbitrary placement elsewhere in the arena.

## Tests

Development test and packaging commands run from the Git checkout; the
competition archive contains the runtime project and documentation.

Run the portable automated tests:

```sh
python3 -m unittest discover -s tests -v
```

The suite includes 100 generated connected maze layouts, with and without cycles,
as well as unique counting, ambiguous majority, camera decoding, collision guards,
odometry, mapping, and locked-asset checks.

On macOS with Webots installed in `/Applications/Webots.app`, run integration tests:

```sh
python3 tests/run_webots.py --scenario reported-start --official-lighting --output output/reported-start-after.json
python3 tests/run_webots.py --scenario maze --output output/maze.json
python3 tests/run_webots.py --scenario chamber --output output/chamber.json
python3 tests/run_webots.py --scenario continuous --official-lighting --output output/continuous.json
python3 tests/run_webots.py --scenario generated-continuous --output output/generated-continuous.json
python3 tests/run_webots.py --scenario generated-maze --blue-majority --output output/generated-blue-maze.json
```

The runner uses the x86_64 Webots binary through Rosetta because the installed
native launcher reports a Qt NEON compatibility error in this environment. It
creates an isolated temporary project and adds a **test-only** supervisor. The
`--official-lighting` option retains the supplied world's original lighting; other
fixtures use built-in lighting to avoid asset-download dependencies. Generated
layouts alter only temporary copies. Submitted arena files and the competition
robot never receive supervisor privileges.

Integration checks measure physical travel, obstacle/wall contacts, full-body
containment, counts, actual LED device states, motor commands, and indication
within three seconds of entering the end cell. Reports go under ignored `output/`.
See `docs/VALIDATION.md` for recorded results and practical limits.

## Submission

```sh
python3 tools/package_submission.py --team YourTeamID
```

This validates the original arena/obstacle hashes and unchanged base mechanics,
then creates `output/YourTeamID_Submission.zip`. Only the outer archive/folder name
changes; required internal names are preserved. All runtime Python helpers are
inside `controllers/main_controller/`. Tests, test supervisors, Git metadata,
logs, caches, and development outputs are excluded.

The physical final's additional section is unspecified in this rulebook and is
outside this simulation implementation. Its separate Final Round Challenge Guide
is required before that behavior can be implemented.
