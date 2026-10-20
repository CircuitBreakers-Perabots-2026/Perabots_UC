# Validation - 20 October 2026

The four final runs below use the same current controller, navigation module and
robot PROTO. `validation-results.json` records their runtime SHA256 fingerprints.
Local detailed logs and individual reports are in ignored `output/`.

## Reported stall regression

The screenshot showed the original saved starting pose and a controller still in
Section 1. The previous controller (`2b2ae8b`) reproduced a stop after **53.76s**,
with 15 seconds of no physical progress. The final controller completes the same
starting pose with all 36 maze cells visited and no contacts. Before/after evidence
is retained in `stall-regression.json`.

The repair accounts for chassis pitch and the axle-to-chassis offset in odometry,
steers and checks arrival at the axle, retains complete detours across intermediate
waypoints, and uses circular corner clearance with the travel margin included in
the map. Scan hits take priority over grazing free-space rays. Local escape from
newly inflated margins avoids moving closer to measured surfaces. During maze
entry and exploration, floor hits and over-wall rays are rejected and grid-aligned
long-range walls correct corridor drift before turns.

## Automated suite

`python3 -m unittest discover -s tests -v`: **31 tests passed**. This includes 100
generated connected maze graphs, half with additional cycles. Added regressions
cover tilted axle tracking, short-target convergence, corner clearance, stable
waypoints and detours, scan-hit precedence, exploration coverage, safe escape,
over-wall entrance rejection, floor-hit rejection, and long-range wall correction.
The suite also checks counts, both majorities, ties, decoding and locked assets.

## Webots R2025a integration

The installed Webots x86_64 binary ran through Rosetta on macOS; Webots launched
Python 3.13.9. The controller requires no third-party Python dependencies.

| Scenario | Simulation time | Physical distance | Contacts | Blue / green | LED delay from first full-body arrival | Result |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Reported saved starting pose, original lighting | 355.49s | 18.69m | 0 | 2 / 3 | 1.57s | PASS |
| Centered continuous start, original lighting | 339.07s | 18.62m | 0 | 2 / 3 | 1.57s | PASS |
| Independent Section 2 chamber start | 237.44s | 14.81m | 0 | 2 / 3 | 1.57s | PASS |
| Changed layout and early end, blue majority | 387.36s | 22.07m | 0 | 3 / 2 | 1.57s | PASS |

Every final run visited **all 36 cells**, finished with its whole body inside the
clear end cell, selected the correct actual LED, commanded both motors to zero,
and remained stationary for another half-second. Drift was below 0.001mm. No
majority LED was lit prematurely, and every indication met the three-second limit.

The temporary test supervisor counts physical obstacle/wall contacts, including
low chassis contacts, while excluding ordinary floor contact. Containment checks
transform chassis corners, conservative wheel/caster bounds and LED lenses by the
measured robot orientation. The harness checks 15 seconds without physical progress
and two seconds entirely outside the track. Test supervisors exist only in temporary
projects and are excluded from the submission.

## Preservation and use

The original Arena.proto, Obstacles.proto, base mechanics, collision shapes,
physics and required LED definitions are unchanged. The permitted-device checks
and packaging validator pass. Submission names remain intact and all runtime
helpers are inside controllers/main_controller/.

After updating controller files, reset the entire simulation in Webots before
running again. Pose estimation and mapping must start at the designated entrance
or start zone, rather than at a previously paused position halfway through a run.

## Limits

These results cover the reported start, centered start, independent maze entrance
and a materially different layout. They do not certify every layout, lighting,
slip condition or arbitrary placement. Start/frame conventions remain documented
in README.md. The original-arena runs retain its lighting; generated and independent
fixtures use built-in lighting. Tied colors and the undisclosed physical-final task
remain subject to the limits documented in REQUIREMENTS.md.
