# ADR 0003: Selected objects and explicit manual positioning

Status: accepted for v0.7.3, 2026-11-15. Owner requested direct object dragging
and correction of color selection that always grasped orange.

The live task previously ignored the editor's selected object. A new recorded
`select_object` command carries an integer scene object index. Single-object goals
follow this index; sorting prioritizes unfinished selected objects and uses stable
receiving slots. Stack order remains fixed. Selection during a closed grasp waits
for the current placement. Repeating the current selection is idempotent.

`grab(kind=object)` now means manual position control, replacing the old 3 N spring.
It preserves the clicked offset and orientation, checks translation on private
MuJoCo data in steps no larger than 2 mm, and holds the last accepted pose.
The free joint's seven position and six velocity entries are explicitly mapped;
robot coordinates and model parameters are not rewritten. Existing contacts may
separate but the proposed move cannot deepen them beyond a 50 micrometre tolerance.
Release/start/stop removes the manual constraint, clears object velocity and restores
ordinary gravity/contact response. The adapter runs before sensor/frame sampling.

This is an intentional change to the live input semantics, not a change to the
physical grasp algorithm. `body` remains bounded external force and continues the
task. Autonomous grasping never uses manual object position control. Records expose
`interaction.mode=direct_position` and `object_hold` events. Height measurements
include manual input, so they cannot by themselves prove an autonomous lift.

Migration: clients requiring force-based object disturbance should keep using
`push(object, force_n)`. Existing `grab/drag/release` field shapes remain accepted.
New selection is live-session state, recorded in actions and world snapshots, not
part of the frozen batch scene schema. Scene reset reconstructs the world; the UI
resends its selected index. Strict historical replay requires the recorded source
version; current code must not claim old spring recordings replay identically.

Single-object receiving x offsets are 0, -0.07, +0.07 m by scene index, so changing
color does not place a block into a previously placed block. Sort offsets retain
their original index-based geometry. Large objects may still obstruct these slots;
existing collision/grasp failures remain visible. Dragging preserves orientation,
does not implement throwing, and uses sampled translational collision checks rather
than continuous collision detection. The batch algorithms, robot assets and model
hashes are preserved and covered by frozen regression tests.
