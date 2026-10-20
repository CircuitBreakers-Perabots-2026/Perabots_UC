# Rulebook traceability

Source: user-supplied University Category rulebook, September 2026 package.
Page numbers below refer to the 20-page PDF, counting its cover as page 1.
Document task requirements are treated as specifications; unrelated registration,
communication, and organizational instructions do not authorize external actions.

| Rule | Implementation | Verification |
| --- | --- | --- |
| Section 1 autonomous obstacle navigation (p.7) | Online ray map, inflated search, clearance guard, opening discovery | Original and generated obstacle fixtures |
| Changed obstacles/walls/tiles; no stored routes (p.9,11) | Sensor-only runtime; no arena-file reads; learned graph and color map | 100 generated graph layouts and generated Webots fixture |
| Maze 9x4, 25cm pitch, 1.2cm walls (p.8-9) | Local grid; wall-based pose correction | Graph and physics tests |
| Count blue/green without duplicate visits (p.8) | Color map keyed by cell | Revisit tests; physical counts vs fixture truth |
| Finish with whole body inside end cell (p.8) | Center waypoint with conservative 6.8cm footprint | Supervisor transforms chassis corners, wheel/caster bounds, and LED lenses into the clear cell |
| Majority LED and stop within 3 seconds (p.8) | Both LEDs off during exploration; final majority LED and zero motor commands in same step | Actual LED/motor readings plus physical entry timing |
| Avoid premature arrival at the end (p.8) | Ahead camera recognizes red and defers its branch | Generated fixture has a terminal near the entrance |
| Max 20x20x20cm (p.10) | Sensors fit inside base's existing horizontal/vertical envelope plus small mount | Geometry review; no added collision or physics shapes |
| Preserve base mechanics and LEDs (p.10) | Only extensionSlot's default sensors changed | SHA256 of entire Robot implementation vs original |
| Permitted devices; no GPS/Emitter/Receiver/Supervisor (p.11) | Cameras, DistanceSensors, IMU, existing encoders | Automated source checks; supervisor FALSE preserved |
| Preserve filenames; helpers in controller folder (p.11-12) | main_controller.py plus navigation.py | Packaging validation |
| Original Arena and Obstacles (p.9,11) | Files byte-for-byte unchanged | SHA256 checks before tests/packaging |
| Independent Section 2 attempts (p.13) | Automatic chamber/first-cell detection at startup | Separate maze and chamber tests |
| No progress for 15 seconds (p.13) | Field replanning and guarded recovery; explicit motion failure | Supervisor watches physical progress |
| Physical final tasks undisclosed (p.14-17) | No invented third-section behavior | Documented scope |

## Unspecified and limiting cases

- A tied or zero/zero tile count has no defined majority in the PDF. The robot
  stops at the terminal with both LEDs off and logs `tie`; it does not invent a
  winning color.
- The solution assumes a reachable maze, the provided entrance/frame convention,
  and a terminal that can be deferred without making remaining branches
  unreachable. An environment requiring passage through its terminal to reach
  unobserved tiles conflicts with a strict three-second stop on first arrival.
- A test fixture cannot certify all possible evaluation layouts, illumination,
  wheel slip, or evaluator placement. Simulation results describe the tested
  cases; hardware transfer and the undisclosed physical-final tasks need
  separate development.
