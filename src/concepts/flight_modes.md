# Flight modes

A flight mode determines how PX4 interprets commands and controls vehicle motion.
Each mode coordinates several parts of the software, and multiple modes can
share controllers. To understand or extend a mode, trace where its commands come
from, how PX4 acts on them, and which conditions allow it to run. Flight mode
behavior depends on vehicle type, as a multicopter holds a location by hovering
while a fixed-wing aircraft flies a loiter pattern.

The examples cover built-in modes and basic Offboard control for
multicopters, fixed-wing aircraft, and vertical takeoff and landing (VTOL)
aircraft. Implementing new modes, including external modes built with Robot
Operating System 2 (ROS 2), is a separate topic.

Source revision: PX4-Autopilot `main`, checked on 2026-11-01 at
[`53b193a6fb1c7aec34fb7334a4ca91d4b9b20332`][revision]. Links are pinned to that
revision so readers can check the explanations as `main` changes.

## What a mode determines

A mode gives meaning to input. Moving a multicopter's pitch stick in Stabilized
mode requests a tilt angle. In Position mode, the same input requests horizontal
motion, which PX4 converts into the tilt and thrust needed to move.

A *setpoint* is a target value passed to a controller, such as a desired attitude
(orientation), angular rate (rotation speed), or position. The controller adjusts
its outputs to bring the measured value toward that target. A mode determines
how setpoints are generated and which controllers act on them.

Table 1 groups modes by the source of their motion commands. Individual modes
can also allow adjustments or pilot takeover.

*Table 1. Mode groups by source of motion commands.*

| Group | Source of motion commands | Examples |
| --- | --- | --- |
| Pilot-controlled | PX4 interprets manual input from a radio controller or joystick. The mode determines the assistance provided. | Acro, Stabilized, Altitude, Position/Cruise |
| Automatic | Onboard behavior generates targets to carry out a task. | Hold, Mission, Return, Takeoff, Land |
| Offboard | An external system supplies setpoints. PX4 runs the controllers appropriate to the requested control level. | Offboard position control or attitude control |

Pilot-controlled modes can provide substantial autopilot assistance. In Altitude
mode, for example, the pilot requests climbs and descents while PX4 controls
vertical motion. The mapping from modes to controllers is defined in
[`getVehicleControlMode()`][control-mode] and the associated
[setpoint-type mapping][setpoint-types].

## How a mode appears in the software

PX4 publishes vehicle state through uORB, its internal publish/subscribe message
system. Two messages connect the selected mode to its control behavior:

- [`VehicleStatus`][vehicle-status] includes `nav_state`, the active flight mode,
  and `nav_state_user_intention`, the requested mode. Its
  `NAVIGATION_STATE_*` constants identify the modes used throughout the code.
- [`VehicleControlMode`][vehicle-control-mode] contains flags indicating which
  kinds of control are enabled, such as attitude, altitude, position, manual
  input, and Offboard control.

The displayed mode names differ from some of the `VehicleStatus` identifiers.
Altitude is `NAVIGATION_STATE_ALTCTL` (value `1`), Position is
`NAVIGATION_STATE_POSCTL` (value `2`), and Hold is
`NAVIGATION_STATE_AUTO_LOITER` (value `4`). The `nav_state` field stores the
numeric value, and the [mode-name mapping][mode-names] defines its displayed name.

The `VehicleControlMode` flags show which kinds of control a mode enables. For
example, Altitude enables attitude and vertical control, while Position also
enables horizontal velocity and position control. Modes that enable the same
controllers can generate different setpoints. For multicopters, Position
interprets pilot input, while Hold generates an automatic hold target.

Arming is recorded separately in `VehicleStatus.arming_state`. A mode can be
selected while disarmed; selection alone does not arm the vehicle or start a
flight.

### A mode coordinates multiple modules

Mode behavior spans several modules, as shown in Table 2. Commander determines
which mode is active, while other modules generate targets and control the
vehicle. To locate a feature, identify which responsibility it changes.

*Table 2. Responsibilities and source entry points for mode behavior.*

| Responsibility | Main source entry point |
| --- | --- |
| Handle mode requests, health checks, and failsafe decisions; publish active state and controller flags. | Commander: [`handleModeIntentionAndFailsafe()`][commander-state] and [`updateControlMode()`][commander-control] |
| Generate navigation targets for automatic tasks such as Mission, Hold, and Return. | Navigator: [selection by `nav_state`][navigator] |
| Select multicopter flight tasks that generate motion setpoints. | Flight mode manager: [`start_flight_task()`][flight-tasks] |
| Select fixed-wing control behavior and generate guidance targets. | Fixed-wing mode manager: [`set_control_mode_current()`][fw-mode-manager] |
| Manage VTOL transitions and combine multicopter and fixed-wing control outputs. | [VTOL attitude control][vtol-control] |

Lower-level controllers track the resulting setpoints, and control allocation
maps thrust and torque requests to actuators. A *flight task* generates motion
setpoints within this process. Multiple automatic modes use the multicopter
`Auto` task, and Position can select different tasks according to `MPC_POS_MODE`.
A change to a shared task or controller can therefore affect several flight
modes.

## Multicopter and fixed-wing behavior

The aircraft's physical capabilities shape what a mode can do. A multicopter can
brake to a stop and hover. A fixed-wing aircraft must keep flying, so maintaining
a location generally means circling it.

Table 3 compares the behavior of representative modes. Parameters and entry
requirements are covered in the linked source documentation.

*Table 3. Representative mode behavior for multicopters and fixed-wing aircraft.*

| Mode | Multicopter | Fixed-wing |
| --- | --- | --- |
| Manual | Uses attitude stabilization. Tools may present this as Stabilized. | Pilot commands pass to control allocation without attitude or rate stabilization. |
| Acro | Sticks request angular rates. Centering the sticks stops rotation; the existing tilt remains. | Sticks request angular rates, allowing aerobatic attitudes. |
| Stabilized | Roll and pitch command attitude; throttle commands thrust. Centered roll and pitch level the aircraft. | Roll and pitch command attitude; throttle commands thrust. Altitude and airspeed remain the pilot's responsibility. |
| Altitude | Adds vertical control. Centered vertical input holds altitude, while horizontal drift remains possible. | Controls altitude or climb/descent while the pilot commands bank angle. The pilot maintains the ground course. |
| Position / Cruise | Position interprets stick input as horizontal motion. Centering the sticks makes the aircraft brake and hold position. | Cruise maintains a ground course and altitude when the sticks are centered. |
| Hold | Automatically stops and hovers at a position. | Automatically flies a loiter pattern around a position. |

The [multicopter overview][mc-overview] and [fixed-wing overview][fw-overview]
describe these mode families. The difference in Manual behavior is explicit in
[`getVehicleControlMode()`][control-mode]: rotary-wing Manual uses attitude
control, while fixed-wing Manual uses thrust and torque requests.

### Example: multicopter Position and Hold

When a pilot moves a multicopter forward in Position mode and then centers the
sticks, PX4 brakes and holds the resulting position. The mode remains Position,
`NAVIGATION_STATE_POSCTL`; stopping is part of that mode's behavior.

Selecting Hold changes the mode to `NAVIGATION_STATE_AUTO_LOITER`. Navigator
provides the automatic navigation target, and the multicopter `Auto` flight task
generates motion setpoints. In Position, the flight mode manager instead selects
a manual position or acceleration task. Both paths use position control, but
they obtain their targets differently. See [flight task selection][flight-tasks]
and [Navigator's mode selection][navigator].

With the default pilot override configuration, moving the sticks in multicopter
Hold can switch the mode to Position, or Altitude if position is unavailable.
Takeover policy governs this mode change. See the
[multicopter Hold behavior][mc-hold].

### Example: fixed-wing Cruise and Hold

In Cruise, centering the sticks lets the aircraft maintain a straight ground
course and altitude. Selecting Hold tells it to loiter around a location. The
aircraft continues flying in both modes. See the [Cruise description][fw-cruise] and
[fixed-wing Hold behavior][fw-hold].

Cruise uses the same internal `NAVIGATION_STATE_POSCTL` identifier as multicopter
Position. The vehicle-aware [mode-name helper][mode-names] calls it "Cruise" for
fixed-wing flight; some tools still call it "Position." Hold uses
`NAVIGATION_STATE_AUTO_LOITER` for both vehicle types. When reading a mode name or
log, interpret the navigation state together with the vehicle type.

## VTOL has a separate flight configuration

A VTOL aircraft records both its flight mode and its flight configuration,
including transitions between multicopter and fixed-wing flight. For example,
it can remain in Mission while hovering, transitioning, or flying as a fixed-wing
aircraft.

[`VtolVehicleStatus.vehicle_vtol_state`][vtol-status] distinguishes multicopter
flight, transition to fixed-wing, fixed-wing flight, and transition to multicopter.
Commander uses this state to update `VehicleStatus.vehicle_type` and the
transition flags in `VehicleStatus`. In this revision, `vehicle_type` reports
rotary-wing during transitions, so the flags are needed to identify a transition.
See
[`Commander::vtolStatusUpdate()`][commander-vtol].

VTOL mode behavior generally follows the current flight configuration, with
additional handling for takeoff, landing, and Return. Transitions also affect
which control paths run: the multicopter flight mode manager can select a
transition task, and fixed-wing mode management can remain active. See the
[VTOL overview][vtol-overview],
[flight task selection][flight-tasks], and [fixed-wing mode management][fw-mode-manager].

### Example: one VTOL mission across a transition

Consider a mission containing a VTOL takeoff, fixed-wing waypoints, and a VTOL
landing. The active mode can remain Mission throughout these phases while the
VTOL state changes, as shown in Table 4.

*Table 4. Flight mode and VTOL state during an example mission.*

| Mission phase | Active flight mode | VTOL state |
| --- | --- | --- |
| Vertical ascent | Mission | Multicopter |
| Transition for forward flight | Mission | Transition to fixed-wing |
| Fly the waypoint route | Mission | Fixed-wing |
| Transition for vertical landing | Mission | Transition to multicopter |
| Vertical descent and landing | Mission | Multicopter |

Mission items advance within Mission mode. A VTOL transition command changes the
flight configuration without necessarily changing `nav_state`. The
[VTOL mission description][vtol-mission] covers these commands, and
[VTOL attitude control][vtol-control] handles transition requests and the
transition state. Changes to transition behavior therefore require considering
both the flight configuration and the active mode.

The [VTOL flight and transitions](vtol_flight_and_transitions.md) chapter
traces the subsystem handoffs, tailsitter transition conditions, and their
implications for a shared rotational controller.

## Mode requirements and failsafe behavior

Modes depend on valid estimates and inputs. Altitude needs a local altitude
estimate; Position adds a horizontal position requirement. Mission needs a valid
mission and the required navigation estimates. These requirements concern
estimator outputs: a position estimate can come from sources other than satellite
navigation.

[`getModeRequirements()`][mode-requirements] records requirements as bitmasks,
with one bit per navigation state. Health and arming checks use them to determine
which modes can run and whether the vehicle can arm in them. A mode can also
explicitly prevent arming: Return and Land are examples.

The checks apply differently as the vehicle's state changes:

1. While disarmed, PX4 generally allows mode selection even if its requirements
   are unmet. Arming checks still apply.
2. While armed, a mode request is normally checked against whether the target
   mode can run. A request can be rejected or use a permitted fallback.
3. If requirements fail during flight, failsafe policy determines the response.

See [`UserModeIntention::change()`][mode-intention] for request handling.
For example, loss of usable horizontal position in Position mode can cause a
fallback to Altitude if its requirements remain satisfied. Further fallback
depends on the remaining estimates, manual control availability, configuration,
and other active failures. The [failsafe fallback logic][failsafe] makes those
decisions.

A fallback can make `nav_state` differ from `nav_state_user_intention`: the
vehicle executes a fallback while still recording the requested mode. When
diagnosing behavior, inspect both fields and the reported health and failsafe
state. A failsafe can also activate the same Return or Land behavior that an
operator can request, so the active mode alone does not identify what caused the
change.

## How modes are switched

Commander handles a mode switch in two stages. It first accepts or rejects a
request to change the intended mode, then resolves that intention against
failsafe policy to select the active mode. Other modules react to the state that
Commander publishes. This separation explains how a request can be accepted
while `nav_state` still differs from `nav_state_user_intention`.

### Who can request a change

The word "user" in `nav_state_user_intention` includes software that requests an
operating mode. Table 5 lists the main request paths. The field records a mode
value, and the identity of the person or process that requested it is not stored
there.

*Table 5. Sources of mode requests and their paths into Commander.*

| Requester | Path into Commander | Example |
| --- | --- | --- |
| Pilot using radio control (RC) switches | Manual control publishes an `action_request`, which Commander handles in `executeActionRequest()`. | Select the mode assigned to a flight-mode slot. |
| Ground station or companion program | A `vehicle_command` reaches `handle_command()`, which translates the command into a navigation-state identifier. | Request Hold, Return, or Offboard. |
| Onboard completion logic | Commander calls `UserModeIntention::change()` when processing a completed task. | Switch to Hold after a standalone takeoff finishes. |
| Registered mode executor | A command on `vehicle_command_mode_executor` is checked for ownership before being forwarded to `vehicle_command`. | A component sequences flight modes as part of an operation. |

See the [RC mode-slot requests][rc-mode-slots],
[RC action handler][commander-rc-actions],
[vehicle-command handler][commander-mode-commands],
[completion handling][commander-completion], and
[executor command forwarding][executor-commands]. For MAVLink `SET_MODE`, the
[MAVLink receiver][mavlink-set-mode] constructs the `vehicle_command` that
Commander consumes.

The [`ModeChangeSource` enum][mode-change-source] groups RC and ordinary vehicle
commands under `User`. `ModeExecutor` identifies executor-originated requests
and is also used by some onboard completion transitions. These categories help
manage executor ownership, and they do not distinguish a human ground-station
operator from an ordinary companion program.

Pilot stick movement provides another request path. For a multicopter or a VTOL
in multicopter flight, configured stick override can request Position while an
automatic or Offboard mode is active. If a failsafe is already active, Commander
passes the takeover request to the failsafe state machine. See
[`manualControlCheck()`][commander-stick-override].

### How competing requests are resolved

Ordinary requests share one stored intention. Each accepted call to
[`UserModeIntention::change()`][mode-intention] replaces it, so processing order
matters. That method applies the same acceptance check to RC and ordinary
vehicle-command requests. In this revision, the
[Commander loop][commander-request-order] handles a `vehicle_command` before an
`action_request`. If both request a mode change in the same pass and both are
accepted, the later action request replaces the command's intention. This is a
consequence of the loop order, and later requests can replace it again.

Acceptance depends on the requested mode's requirements, as described in
[mode requirements and failsafe behavior](#mode-requirements-and-failsafe-behavior).
While armed, `change()` normally checks `canRun()`. Selected callers can allow a
fallback or use `force` to skip that check, but the failsafe stage still resolves
the active mode. For example, the mode-command handler forces a Land request so
that a fallback can be selected if Land cannot run. Once Termination is active,
`change()` refuses requests to leave it, even when `force` is set.

Registered mode executors have an additional ownership check. Commander forwards
commands from their dedicated topic only when the sending executor is currently
in charge. A `User` request for a built-in mode without an executor owner returns
ownership to PX4's default autopilot executor. See
[executor command forwarding][executor-commands] and
[`onUserIntendedNavStateChange()`][executor-ownership].

Failsafe arbitration then determines whether the intended mode can become
active. When multiple failsafe actions are active, the framework selects the
highest-priority action in its [`Action` ordering][failsafe-actions]. It also
applies delays, clearing conditions, and takeover restrictions before selecting
an executable response. A new mode request can signal pilot takeover, but that
takeover must be permitted by the active failsafe policy. See
[`FailsafeBase::getSelectedAction()`][failsafe-selection].

RC mode selection usually responds to a switch change. After initialization,
leaving the mode-slot switch in one position does not repeatedly request that
mode on every RC update. A ground-station request can therefore remain active
while the RC switch stays in its previous position. See
[`ManualControl::processSwitches()`][rc-mode-switches].

### What happens after a request is accepted

The switch proceeds through shared state and module-specific behavior:

1. `UserModeIntention::change()` stores the intended mode, sets
   `_had_mode_change`, and notifies the mode-management handler so executor
   ownership can be updated. It also remembers a suitable mode to restore after
   disarming.
2. [`handleModeIntentionAndFailsafe()`][commander-state] evaluates failsafes and
   sets `nav_state_user_intention` and `nav_state`. With no overriding failsafe
   action, the active mode follows the intention. When `nav_state` changes,
   Commander updates `nav_state_timestamp`.
3. Commander derives `VehicleControlMode` from the active mode and publishes it
   with `VehicleStatus`. The [publication path][commander-state-publication]
   supplies the state that other modules consume.
4. Those modules update their behavior. Navigator selects an automatic behavior
   and runs its activation or inactivation hooks. The multicopter flight mode
   manager selects a flight task, and fixed-wing mode management selects its
   control behavior.

The consumers have their own transition logic. In Navigator,
[`NavigatorMode::run()`][navigator-lifecycle] calls `on_activation()` when a
behavior becomes active and `on_inactivation()` when it stops. For a new
multicopter flight task, [`switchTask()`][flight-task-switching] saves the previous
trajectory setpoint and passes it to the new task's `activate()` method. If the
selected task is already running, it returns without reactivating it. Several
automatic modes share the `Auto` task, so a navigation-state change can update
Navigator's behavior while that flight task continues running.

Command acceptance and active-mode selection occur at different points in the
Commander loop. In this revision, command and RC action handling occur after
`handleModeIntentionAndFailsafe()`, so an accepted request normally reaches that
resolver on the next pass. The command acknowledgment reports whether the
request was accepted. To determine what is running, inspect the subsequently
published `nav_state` and failsafe state.

An accepted request for the current intention still sets `_had_mode_change`.
This lets a repeated request signal takeover during a failsafe. It does not by
itself change `nav_state_timestamp`, which updates only when the active mode
changes.

### Example: a ground station requests Hold

Suppose a multicopter is flying in Position, its estimates are valid, and no
failsafe is active. The RC mode-slot switch remains in Position, and the pilot
keeps the sticks centered. A ground station sends an accepted request for Hold.
Commander stores `NAVIGATION_STATE_AUTO_LOITER` as the intention, then selects
it as the active mode on the next resolution pass.

The published controller flags enable automatic control. Navigator activates
its loiter behavior, whose [`on_activation()`][loiter-activation] initializes the
hold target. The multicopter flight mode manager switches from its manual task
to `Auto`, which generates motion setpoints from the navigation target. Position
control remains enabled throughout this change.

The unchanged RC switch does not immediately undo the ground-station request.
If the pilot moves it to an Altitude slot, manual control publishes a new action
request. If accepted, that request replaces the intention with
`NAVIGATION_STATE_ALTCTL`, and the same resolution and publication process runs
again.

## Basic Offboard behavior

Offboard uses setpoints supplied by an external system through supported MAVLink
messages or ROS 2 topics. The requested control level determines which PX4
controllers remain active. For example, with multicopter position setpoints,
PX4 retains position control and the inner control loops. With attitude
setpoints, PX4 retains attitude and rate control while the external system
determines the desired attitude.
See the [Offboard branch of the controller mapping][control-mode].

PX4 requires a stream of recent Offboard control messages before arming in
Offboard or entering the mode while armed. It also checks the estimates needed
for the requested control level. If that stream is lost, `COM_OF_LOSS_T` sets the
loss timeout, and `COM_OBL_RC_ACT` selects the response, subject to failsafe
handling.
The [Offboard health check][offboard-check] and
[Offboard documentation][offboard-docs] describe this behavior.

MAVLink setpoint messages carry both the target and evidence that the external
system is still sending messages. With ROS 2, `OffboardControlMode` selects the
control level and acts as a keepalive: a periodic message indicating that the
external system is still running. Separate topics carry setpoints, so a live
keepalive alone does not establish that a setpoint publisher is still updating.

Supported setpoint types and coordinate frames depend on the interface and
vehicle. Check support for the intended vehicle before adapting an Offboard
example from another vehicle type.

## Finding the right place for a feature

Locate a feature by identifying which responsibility changes. Input
interpretation or motion shaping usually belongs in flight tasks or
vehicle-specific mode management. Automatic route behavior belongs in Navigator.
Mode requirements and fallback policy belong in Commander's health and failsafe
checks. Controller response and actuator mapping belong farther down the control
path.

For example, to change how a multicopter brakes when the pilot centers the
Position sticks, examine the selected manual flight task. To change the
destination used by Return, examine Navigator's Return implementation. To change
when either mode can run, examine the mode requirements and health checks.

[revision]: https://github.com/PX4/PX4-Autopilot/commit/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332
[vehicle-status]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/msg/versioned/VehicleStatus.msg
[vehicle-control-mode]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/msg/VehicleControlMode.msg
[control-mode]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/ModeUtil/control_mode.cpp#L46-L163
[setpoint-types]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/ModeUtil/setpoint_types.cpp#L46-L166
[commander-state]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/Commander.cpp#L2547-L2622
[commander-control]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/Commander.cpp#L2809-L2828
[navigator]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/navigator/navigator_main.cpp#L1005-L1083
[flight-tasks]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/flight_mode_manager/FlightModeManager.cpp#L140-L283
[fw-mode-manager]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/fw_mode_manager/FixedWingModeManager.cpp#L380-L530
[vtol-control]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/vtol_att_control/vtol_att_control_main.cpp
[mc-overview]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/docs/en/flight_modes_mc/index.md
[fw-overview]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/docs/en/flight_modes_fw/index.md
[mc-hold]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/docs/en/flight_modes_mc/hold.md
[fw-cruise]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/docs/en/flight_modes_fw/cruise.md
[fw-hold]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/docs/en/flight_modes_fw/hold.md
[mode-names]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/lib/modes/ui.hpp#L73-L125
[vtol-status]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/msg/versioned/VtolVehicleStatus.msg
[commander-vtol]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/Commander.cpp#L2405-L2435
[vtol-overview]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/docs/en/flight_modes_vtol/index.md
[vtol-mission]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/docs/en/flight_modes_vtol/mission.md
[mode-requirements]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/ModeUtil/mode_requirements.cpp
[mode-intention]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/UserModeIntention.cpp#L44-L95
[failsafe]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/failsafe/failsafe.cpp#L770-L825
[offboard-check]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp#L38-L117
[offboard-docs]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/docs/en/flight_modes/offboard.md
[mode-change-source]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/UserModeIntention.hpp#L39-L67
[rc-mode-slots]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/manual_control/ManualControl.cpp#L457-L512
[rc-mode-switches]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/manual_control/ManualControl.cpp#L197-L339
[commander-rc-actions]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/Commander.cpp#L1872-L1954
[commander-mode-commands]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/Commander.cpp#L933-L1101
[commander-completion]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/Commander.cpp#L2252-L2279
[executor-commands]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/Commander.cpp#L1783-L1818
[mavlink-set-mode]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/mavlink/mavlink_receiver.cpp#L1120-L1147
[commander-stick-override]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/Commander.cpp#L3164-L3200
[commander-request-order]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/Commander.cpp#L2070-L2150
[executor-ownership]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/ModeManagement.cpp#L465-L485
[failsafe-actions]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/failsafe/framework.h#L52-L76
[failsafe-selection]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/failsafe/framework.cpp#L444-L622
[commander-state-publication]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/Commander.cpp#L2166-L2184
[navigator-lifecycle]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/navigator/navigator_mode.cpp#L56-L85
[flight-task-switching]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/flight_mode_manager/FlightModeManager.cpp#L382-L417
[loiter-activation]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/navigator/loiter.cpp#L54-L75
