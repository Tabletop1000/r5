# Lidar collision avoidance

This tutorial adds the Waveshare D500 lidar kit to R5 and uses it to stop the robot before it hits
people, animals or equipment. Work through the steps in order. Each one ends with a check that the
next step relies on.

| Step | File | Result |
| --- | --- | --- |
| 1 | [Hardware and driver](01-hardware-and-driver.md) | The D500 on the Pi's USB, a stable `/dev/ttyLidar`, and `/scan` on the bench |
| 2 | [Robot model and launch](02-robot-model-and-launch.md) | The lidar in the URDF, the robot's own body filtered out, and a dashboard alert if the lidar goes quiet |
| 3 | [Simulation](03-simulation.md) | The lidar and obstacles in Gazebo, for trying the rest on the laptop |
| 4 | [Collision monitor](04-collision-monitor.md) | Nav2 commands slowed or stopped when something is close |
| 5 | [Jobs and the dashboard](05-jobs-and-dashboard.md) | A spray job that turns the sprayer off, waits for the way to clear, then carries on |
| 6 | [Field testing](06-field-testing.md) | Stopping distance measured, the failure modes tested, and the zones tuned |
| 7 | [Driving around obstacles (optional)](07-driving-around-obstacles.md) | Why R5 stops rather than steering around, and what changing that would take |

## The approach

Most of the time the lidar sees nothing nearby, so the design aims at one job: **stop, wait, and
carry on**. The robot doesn't plan around obstacles. People and animals usually move out of the way.
If something stays in the way, it's probably equipment, and an operator should look at it.

```
                       ┌─────────────┐   ┌───────────────┐
D500 ──USB──► ldlidar ─► /scan_raw ──► laser_filters ─► /scan
                       └─────────────┘   (robot body    │
                                          removed)      │
                                                        ▼
controller_server ─► velocity_smoother ─► collision_monitor ─► /mux/nav ─► twist_mux ─► /cmd_vel
                                           │  stop zone: zero velocity
                                           │  slowdown zone: 30 % speed
                                           └► /collision_monitor_state ─┐
                                                                        ▼
                                          collision_detector ──────► job_manager
                                          (is the way clear yet?)    sprayer off, cancel goal,
                                                                     wait, resend
```

There are two layers, and each works without the other:

1. **`collision_monitor`** (Nav2) sits between the velocity smoother and `twist_mux`. It checks every
   command against the latest scan, with no costmap and no planning. It's a short, simple path, so
   it's the part to trust.
2. **`job_manager`** reacts to the stop. It turns the sprayer off, cancels the Nav2 goal so the
   progress checker doesn't fail the job, waits until the way is clear, and sends the goal again.
   If the way stays blocked, it pauses the job and raises an alert.

The Xbox controller, keyboard teleop and motor test don't pass through the monitor. A person is
holding the deadman button, or the wheels are off the ground. [Step 4](04-collision-monitor.md#what-it-guards)
explains this choice and how to change it.

## What the lidar can't do

Plan around these limits rather than discovering them in the field:

- **Anything below the scan plane.** The D500 scans one horizontal plane, about 0.36 m above the
  ground where the URDF has it now. A lamb or dog lying down, a person sitting or lying, a rock or a
  low trailer drawbar can all sit under it. Step 1 discusses the mounting height.
- **Wire fences.** You're right to be sceptical. At 0.72° between beams, the beams are about 4 cm
  apart at 3 m, and a 2.5 mm wire fills about 6 % of that gap. Most scans miss it, and a strand only
  shows at all if it happens to be at scan height. Posts do show, as isolated points. Wide white
  electric fence tape does show. Keep relying on the field boundary and GPS for fences. That's
  another reason the robot stops rather than steering around: a detour is the only way it could
  leave its planned path towards a fence.
- **Tall grass or crops.** If the vegetation reaches the scan plane, the lidar sees obstacles
  everywhere and the robot never moves. Check the height of what R5 will drive through before
  choosing a mounting height.
- **Spray mist, rain and dust** return points close to the robot. The polygons' `min_points` and
  the body filter handle some of this. Step 6 tests it with water.
- **Direct sun.** The D500 is rated to 60 klx, and full summer sun reaches about 100 klx. Expect
  more noise at midday with the sun low in front of the robot.
- **It isn't a safety-rated sensor.** This is collision avoidance for a slow robot, not a certified
  safety function. Keep the dashboard Stop button and supervision as they are.

## Related gap: motor commands with no timeout

The motor node firmware ignores short messages and has no command timeout. If `ros2_control_node`
stops sending, `diff_drive_controller`'s 0.5 s `cmd_vel_timeout` doesn't help, because the ESP32s
keep driving at the last speed. The collision monitor only works while the chain from Nav2 to the
motors is alive. A firmware watchdog that stops the motors after about 0.5 s without a command
would close this gap. It isn't part of this tutorial, but do it before relying on the lidar.

## Before you start

- **Nav2 must drive the real robot.** The robot currently stops after about 3 s of a Nav2 goal and
  then fails the progress check. Fix that first. Until then, you can test the collision monitor
  with teleop, as step 4 describes.
- **Memory.** This adds four nodes, all C++: the driver, the scan filter, the collision monitor and
  the collision detector. Note the dashboard's memory figure before step 2 and after step 5.
- **Software:** `ros-jazzy-laser-filters` and `ros-jazzy-nav2-collision-monitor` from apt, plus the
  LDROBOT driver built from source.
