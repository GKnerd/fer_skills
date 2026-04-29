# fer_skills — Design

A skill library for the Franka Emika Robot (FER) running in MuJoCo + ROS 2 Control + MoveIt 2.

## Goal

Expose high-level robot behaviors ("skills") as ROS 2 actions so that any caller — Python script, behavior tree, web UI, planner — can invoke them without touching MoveIt internals.

Initial skills:
- `GoHome` — drive the arm to the SRDF "ready" pose
- `MoveToPose` — Cartesian goal for the TCP
- `PickObject` — top-down pick at an `(x, y, z)` point

## Why ROS 2 Actions

Skills are motions. They take seconds, can fail mid-execution, and benefit from cancellation and progress feedback. Services have no cancel and no feedback. Plain library calls force callers into our process. Actions give us:

- Cancellation
- Progress feedback
- Goal lifecycle (accepted / executing / succeeded / aborted / canceled)

One node — `skill_server_node` — hosts all the action servers.

## Why MoveIt Task Constructor (MTC)

MTC is the path of least resistance for composite skills like pick-and-place. It provides built-in stages (`GenerateGraspPose`, `Connect`, `MoveTo`, `MoveRelative`, …) that already solve the problems we'd otherwise reinvent: searching across grasp candidates, propagating planning constraints, computing approach/retreat motions.

Using MTC saves weeks of work on `PickObject` alone.

**What MTC does not do: online replanning.** MTC is a task planner — it builds a complete solution before execution. Once execution starts, MTC is out of the loop.

## Replanning strategy

Replanning is a separate problem from skill composition. The plan is two-phase:

**Phase 1.5 — Re-plan on failure.** Inside each action server, catch execution failure and re-invoke the planner from current state. Cheap to implement, gets us 80% of the value.

**Phase 2 — Online replanning via MoveIt Hybrid Planning.** When we need real reactivity (dynamic obstacles, moving targets), wrap MTC as the *Global Planner* inside MoveIt's Hybrid Planning architecture. The Local Planner runs at controller rate and triggers global replans on demand. This is the right tool — MTC and Hybrid Planning are complementary, not competing.

## Architecture

```
┌─────────────────────────────────────────┐
│  fer_skills action servers              │  ← public surface
│  (GoHome, MoveToPose, PickObject, …)    │
├─────────────────────────────────────────┤
│  Skill primitives library               │  ← reusable building blocks
│  (motion_primitives, gripper_primitives)│
├─────────────────────────────────────────┤
│  MoveGroupInterface  +  MTC  +  ........│  ← MoveIt 2
│  control_msgs/GripperCommand            │  ← gripper_effort_controller
└─────────────────────────────────────────┘
```

Trivial skills (`GoHome`) bypass MTC and call `MoveGroupInterface` directly. Composite skills (`PickObject`) use MTC. The action surface is uniform either way — callers don't know or care.

## Why keep our own action layer on top of MTC

- Callers depend on stable types (`fer_skills/action/PickObject`), not `moveit_task_constructor_msgs`.
- We can swap MTC for something else later without breaking clients.
- Trivial skills can skip MTC entirely — wrappers hide this.
- One stable interface for behavior trees, scripts, and supervisors.

## Package layout

```
fer_skills/
├── DESIGN.md
├── package.xml
├── CMakeLists.txt
├── action/
│   ├── GoHome.action
│   ├── MoveToPose.action
│   └── PickObject.action
├── include/fer_skills/
│   └── skill_server_node.hpp
├── src/
│   ├── skill_server_node.cpp        # main(), action server registration
│   └── skills/
│       ├── go_home.cpp              # uses MoveGroupInterface directly
│       ├── move_to_pose.cpp         # stub for now
│       └── pick_object.cpp          # stub; will use MTC
├── launch/
│   └── fer_skills.launch.py
└── config/
    └── default_params.yaml          # group names, default speeds, approach height
```

## Conventions

- **Planning groups:** `fer_arm` (7-DoF arm), `fer_hand` (gripper). From `franka_description`'s SRDF.
- **Named states:** `ready` (arm), `open` / `close` (hand). From SRDF.
- **TCP frame:** `fer_hand_tcp`.
- **Default planning frame:** `base`.
- **Pose inputs:** always `geometry_msgs/PoseStamped` so callers specify their frame; the server transforms into the planning frame.
- **Default grasp orientation:** top-down (gripper Z-axis aligned with −world Z) when caller doesn't specify.

## Failure semantics

Action results return distinct codes so behavior trees can react:

- `success = true` — skill completed
- `success = false, message = "PLANNING_FAILED"` — planner couldn't find a solution
- `success = false, message = "EXECUTION_FAILED"` — controller rejected/aborted the trajectory
- `success = false, message = "CANCELLED"` — caller canceled
- `success = false, message = "INVALID_GOAL"` — bad inputs (out of workspace, etc.)

## Build order

1. Action gen + package skeleton — verify `skill_server_node` builds and runs empty.
2. `GoHome` via `MoveGroupInterface::setNamedTarget("ready")`. Smallest possible end-to-end test.
3. `MoveToPose` via `MoveGroupInterface::setPoseTarget`.
4. Wrap gripper as `control_msgs/action/GripperCommand` client.
5. `PickObject` using MTC's pick template (`GenerateGraspPose` + `Connect` + `MoveRelative` for approach/lift).
6. Add re-plan-on-failure inside servers (Phase 1.5).
7. Hybrid Planning integration when we actually need online replanning (Phase 2).

## Open decisions

- **Gripper API surface:** wrap as separate `GripperOpen` / `GripperClose` actions, or keep gripper internal to composite skills only? Likely both.
- **Planning scene management:** does `fer_skills` own the scene, or does a separate node? Lean toward separate.
- **Velocity / acceleration scaling defaults:** 0.5 each is a reasonable start.
