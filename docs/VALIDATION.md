# Validation - 20 October 2026

All results below use the current controller, navigation module, and robot PROTO.
The raw evidence in `validation-results.json` includes SHA256 fingerprints of the
runtime files. Local run logs and individual reports are in ignored `output/`.

## Automated suite

`python3 -m unittest discover -s tests -v`: **18 tests passed**. One test exercises
100 generated connected maze graphs, half with additional cycles, with changed
wall connections, terminal positions and colored dead ends. Other tests cover
unique counting, both majorities, ties, sensor decoding, odometry, entrance
alignment, motion guards, terminal deferral, and locked-component checks.

## Webots R2025a integration

The installed Webots x86_64 binary ran through Rosetta on macOS; Webots launched
Python 3.13.9. No third-party controller Python dependencies were installed.

| Scenario | Simulation time | Physical distance | Contacts | Blue / green | LED delay from first full-body arrival | Result |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Original continuous run, original lighting | 317.79s | 18.24m | 0 | 2 / 3 | 1.44s | PASS |
| Independent Section 2 chamber start | 243.30s | 14.60m | 0 | 2 / 3 | 1.44s | PASS |
| Changed obstacles, walls, tiles and terminal; blue majority | 348.64s | 21.11m | 0 | 3 / 2 | 1.57s | PASS |

Every scenario visited **all 36 cells**, finished with the entire body inside the
clear terminal cell, selected the correct actual LED device, commanded both
motors to zero, and remained stationary for a further half-second. Measured drift
was below 0.001mm. No decision LED was lit prematurely. Each LED delay was below
the three-second rulebook deadline, including the changed fixture's early terminal.

The test-only supervisor counts physical contacts against obstacle/wall footprints,
including low chassis contacts; it excludes ordinary floor contacts. Body checks
transform chassis corners, conservative wheel/caster bounds, and LED lenses by the
robot's measured orientation. The harness also detects 15 seconds without physical
progress and two seconds entirely outside the track. It exists only in temporary
test projects and is excluded from the submission archive.

Additional development runs separately verified a first-black-cell maze start and
a generated blue-majority maze. The table above records the final runtime's
continuous and independent-chamber regression checks.

## Submission and preservation

- `Arena.proto` and `Obstacles.proto` match the original SHA256 hashes.
- The entire base `Robot` implementation, including chassis, drivetrain, physics,
  collision geometry, and required LED nodes, matches its original SHA256 hash.
- Additions are default sensor nodes inside the existing extensionSlot; Supervisor
  remains FALSE. No GPS, Emitter, Receiver, or Lidar device was introduced.
- The packaging validator checks these invariants before writing an archive.
- The archive preserves all required names and places runtime helpers inside the
  main controller folder. Test supervisors, logs, caches and Git metadata are absent.

## Limits

These tests establish behavior for the original arena and a materially different
fixture, with broader graph-level coverage. They do not certify every possible
physical layout, illumination, slip condition, or starting displacement. The fixed
start/entrance/frame convention is documented in README.md. The generated fixture
uses built-in lighting; the original continuous case retains the supplied world's
lighting. The rulebook does not define a tied-color indication or disclose the
physical final's additional task; those cases are documented in REQUIREMENTS.md.
