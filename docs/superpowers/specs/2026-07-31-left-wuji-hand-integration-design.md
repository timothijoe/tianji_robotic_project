# Left Wuji Hand Integration Design

## Objective

Attach the original 20-actuator left Wuji Hand model to the simulated robot's
left wrist while preserving the existing right-arm chopping behavior and all
real-robot code. The resulting simulation must remain self-contained and must
not depend on either downloaded repository in `/home/linux/august_folder`.

## Scope

This change covers the MuJoCo simulation only. It adds the hand model, an
adjustable wrist adapter, a small position-control interface, a stable default
open pose, tests, and a visual demonstration. It does not integrate
`wuji-mjlab`, reinforcement-learning policies, grasp planning, tactile sensing,
or physical-hand communication.

## Asset Ownership and Layout

The required original left-hand MJCF meshes will be copied from
`wuji-description/hand/body` into
`robot_assets/mujoco/wuji_hand/`. The corresponding upstream license and an
attribution note will be stored beside the assets.

The copied asset is the project's build-time source of truth. Runtime code must
not read from `/home/linux/august_folder/wuji-description` or
`/home/linux/august_folder/wuji-mjlab`.

The hand description will stay separate from the main chopping scene so the
robot scene remains readable. Asset names and paths will be normalized only as
needed for composition; joint limits, inertial data, collision geometry,
actuator gains, and the upstream 20-joint ordering will be preserved.

## Mechanical Composition

The hand will be attached beneath `left_link7` through a fixed
`left_hand_mount` body. This body is the only place where the flange-to-palm
translation and orientation are defined. The palm and all finger bodies remain
descendants of that mount, so moving the seven left-arm joints moves the entire
hand.

The mount transform will be calibrated in the existing chopping scene. Its
initial orientation must present the palm and fingers naturally, keep the hand
clear of the wrist and table, and avoid obvious mesh intersections. No new
physical degree of freedom is added at the adapter.

## Control Architecture

The existing 14 arm position actuators retain their names and behavior. The hand
adds 20 position actuators, producing 34 actuators in the composed model.

Arm control must continue selecting actuators by name rather than assuming that
all `model.nu` entries belong to the arms. A focused left-hand controller will
own only the 20 upstream hand actuator names and expose:

- the ordered joint and actuator names;
- a default natural-open target;
- validation and application of a complete 20-value position target;
- a convenience operation to restore the default open pose.

Targets must be finite, have exactly 20 values, and lie within the MuJoCo
actuator control ranges. Invalid targets fail before changing `data.ctrl`.
Partial per-finger command APIs are outside this iteration.

## Initialization and Runtime Behavior

On simulation reset, both arms keep their current initialization behavior and
the hand controller applies its natural-open target. The chosen pose will be
inside every joint limit and will not use a blanket all-zero assumption.

The right arm's chopping and lateral line-following demonstrations remain
behaviorally unchanged. The left arm holds its current configured pose, with
the attached hand following it. This iteration demonstrates hand articulation
through a slow open-to-relaxed-close-to-open sequence; it does not attempt a
grasp.

## Collision and Numerical Stability

The original external palm and finger collision geometry is retained. Existing
upstream collision filtering is preserved. Additional exclusions may be added
only for directly adjacent hand/wrist bodies if inspection shows unavoidable
rest-pose self-contact caused by the attachment.

The composed model must load without duplicate names, invalid mesh paths, or
compiler conflicts. A short headless rollout at the default pose must keep
joint positions and velocities finite and must not show numerical instability.
The initial pose must have no hand-table contact and no unintended hand-arm
penetration.

## Validation

Automated tests will verify:

1. The model loads with 14 arm joints plus 20 hand joints and 34 position
   actuators.
2. All existing arm joint and actuator names remain available.
3. All 20 hand joints and actuators exist in the upstream order.
4. The palm is a descendant of `left_link7` through `left_hand_mount`.
5. The hand controller applies its default pose and rejects wrong-sized,
   non-finite, and out-of-range targets without partial writes.
6. A headless stabilization rollout remains finite and begins without
   unintended hand-table contact.
7. The complete existing simulation test suite and protected real-code checks
   continue to pass.

After automated verification, the interactive MuJoCo viewer will show the
mounted hand and a slow articulation sequence for visual inspection of mount
orientation, clearance, and finger motion.

## Compatibility and Non-Goals

The 20 hand joints retain the original Wuji Hand naming and ordering so a later
adapter can consume `wuji-mjlab` targets. `wuji-mjlab` itself is not installed
or imported because its NVIDIA/Isaac Lab training stack is unnecessary for
native MuJoCo composition.

Files used by the physical robot remain untouched. No existing archive or
legacy simulation files are deleted.
