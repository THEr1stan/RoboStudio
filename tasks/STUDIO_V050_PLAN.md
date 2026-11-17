# v0.5 interactive manipulation studio

Authorized by owner: implement the first five product areas and redesign the local
UI using the visual principles of Apple's iPhone site. Preserve v0.4 as a baseline.

Observable delivery: editable rigid-body scene; physical stepping and teleoperation;
multi-object pick/place and stacking with observation/verification/retry; traced IK
and collision-checked planning; virtual RGB/depth camera and contact feedback;
matched experiments, recorded observations and deterministic replay.

Files: additive studio_scene.py, studio_sensor.py, studio_planning.py,
studio_runtime.py and studio_api.py; studio HTML/CSS/JS and native WebGL renderer;
workbench asset/routes; small backwards-compatible Cell XML injection and IK
orientation argument; tests, benchmark script, README/delivery/architecture docs.

Contracts before implementation: reject malformed scene/sensor/actions; keep model,
state/reference/command frames and hashes; sensor estimates derived from image/depth
only; seeded sampling/noise; collision checks never mutate live state; motion through
actuator effort only; impulses explicitly logged; outcomes measured from engine;
same case/config except tested algorithm; old baseline tests remain passing.

No new dependencies or external robot assets. No publication or irreversible changes.
Robot geometry is the project-authored idealized model, not a calibrated commercial
robot. Camera is an explicitly labelled ray-cast RGB/depth simulation. Physics pauses
between requested step batches; no claim of hard real-time behavior.
