# Flight modes

A flight mode determines how PX4 interprets commands and controls vehicle motion.
For a developer, it connects the source of commands, the controllers that act on
them, and the conditions required for that behavior to run. Vehicle type matters:
holding position means hovering for a multicopter and flying a loiter pattern for
a fixed-wing aircraft.

This chapter introduces built-in modes and basic Offboard behavior for
multicopters, fixed-wing aircraft, and vertical takeoff and landing (VTOL)
aircraft. It provides a starting point for finding feature code. Implementing new
modes and ROS 2 external modes are separate topics.

Source baseline: PX4-Autopilot `main`, checked on 2026-11-01 at
[`53b193a6fb1c7aec34fb7334a4ca91d4b9b20332`][revision]. Links are pinned to that
revision so the explanations remain traceable as `main` changes.

## What a mode determines

A mode gives meaning to input. Moving a multicopter's pitch stick in Stabilized
mode requests a tilt angle. In Position mode, it requests horizontal motion,
which PX4 turns into the tilt and thrust needed to move. A *setpoint* is a target
value passed to a controller, such as a desired attitude, angular rate, or
position.

Table 1 groups modes by where commands come from. These groups are useful for
understanding behavior; individual modes can also allow adjustments or pilot
takeover.

*Table 1. Mode groups by source of motion commands.*

| Group | Source of motion commands | Examples |
| --- | --- | --- |
| Pilot controlled | PX4 interprets manual input from a radio controller or joystick. The mode determines the amount of assistance. | Acro, Stabilized, Altitude, Position/Cruise |
| Automatic | Onboard behavior generates targets to carry out a task. | Hold, Mission, Return, Takeoff, Land |
| Offboard | An external system supplies setpoints. PX4 runs the controllers appropriate to the requested control level. | Offboard position control or attitude control |

Manual input can therefore coexist with substantial autopilot control. In
Altitude mode, for example, the pilot requests climbs and descents while PX4
controls the vertical motion. The mode-to-controller mapping is defined in
[`getVehicleControlMode()`][control-mode] and the associated
[setpoint-type mapping][setpoint-types].

## How a mode appears in the software

PX4 publishes vehicle state through uORB, its internal publish/subscribe message
system. Two messages are especially useful when following a mode through the
code:

- [`VehicleStatus`][vehicle-status] includes `nav_state`, the active flight mode,
  and `nav_state_user_intention`, the requested mode. Its
  `NAVIGATION_STATE_*` constants identify the modes used throughout the code.
- [`VehicleControlMode`][vehicle-control-mode] contains flags indicating which
  kinds of control are enabled, such as attitude, altitude, position, manual
  input, and Offboard control.

For example, Altitude enables attitude and vertical control, while Position also
enables horizontal velocity and position control. Several modes can enable the
same controllers while generating different setpoints for them. Position and
Hold both control a multicopter's position, but Position interprets pilot input
and Hold generates an automatic hold target.

Mode selection and arming are separate state variables. `VehicleStatus.arming_state`
records whether the vehicle is armed. A mode can be selected while disarmed;
selection alone does not arm the vehicle or start a flight.

### A mode coordinates multiple modules

The implementation is spread across policy, setpoint generation, and control,
as shown in Table 2. This division helps locate a change without assuming each
mode has one corresponding class.

*Table 2. Responsibilities and source entry points for mode behavior.*

| Responsibility | Main source entry point |
| --- | --- |
| Handle mode requests, health checks, and failsafe decisions; publish active state and controller flags. | Commander: [`handleModeIntentionAndFailsafe()`][commander-state] and [`updateControlMode()`][commander-control] |
| Generate navigation targets for automatic tasks such as Mission, Hold, and Return. | Navigator: [selection by `nav_state`][navigator] |
| Select multicopter flight tasks that generate motion setpoints. | Flight mode manager: [`start_flight_task()`][flight-tasks] |
| Select fixed-wing control behavior and generate guidance targets. | Fixed-wing mode manager: [`set_control_mode_current()`][fw-mode-manager] |
| Manage VTOL transitions and combine multicopter and fixed-wing control outputs. | [VTOL attitude control][vtol-control] |

Lower-level controllers track the resulting setpoints, and control allocation
maps thrust and torque requests to actuators. A *flight task* is one part of this
implementation: multiple automatic modes use the multicopter `Auto` task, and
Position can select different tasks according to `MPC_POS_MODE`. A change to a
shared task or controller can therefore affect several flight modes.

## Multicopter and fixed-wing behavior

The aircraft's physical capabilities shape what a mode can do. A multicopter can
brake to a stop and hover. A fixed-wing aircraft must keep flying, so maintaining
a location generally means circling it.

Table 3 compares representative modes. It describes behavior rather than every
parameter or entry requirement.

*Table 3. Representative mode behavior for multicopters and fixed-wing aircraft.*

| Mode | Multicopter | Fixed-wing |
| --- | --- | --- |
| Manual | Uses attitude stabilization. Tools may present this as Stabilized. | Pilot commands pass to control allocation without attitude or rate stabilization. |
| Acro | Sticks request angular rates. Centering them stops rotation; the existing tilt remains. | Sticks request angular rates, allowing aerobatic attitudes. |
| Stabilized | Roll and pitch command attitude; throttle commands thrust. Centered roll and pitch level the aircraft. | Roll and pitch command attitude; throttle commands thrust. Altitude and airspeed remain the pilot's responsibility. |
| Altitude | Adds vertical control. Centered vertical input holds altitude, while horizontal drift remains possible. | Controls altitude or climb/descent while the pilot commands bank angle. It does not hold a ground course. |
| Position / Cruise | Position interprets stick commands as horizontal motion and brakes to a position hold when released. | Cruise maintains a ground course when sticks are centered, with altitude control. |
| Hold | Automatically stops and hovers at a position. | Automatically flies a loiter pattern around a position. |

The [multicopter overview][mc-overview] and [fixed-wing overview][fw-overview]
describe these mode families. The Manual distinction is explicit in
[`getVehicleControlMode()`][control-mode]: rotary-wing Manual uses attitude
control, while fixed-wing Manual uses thrust and torque requests.

### Example: multicopter Position and Hold

Imagine moving a multicopter forward in Position mode and then centering the
sticks. PX4 brakes and holds the resulting position. The mode remains Position,
`NAVIGATION_STATE_POSCTL`; stopping is part of that mode's behavior.

Selecting Hold changes the mode to `NAVIGATION_STATE_AUTO_LOITER`. Navigator
provides the automatic navigation target, and the multicopter `Auto` flight task
generates motion setpoints. In Position, the flight mode manager instead selects
a manual position or acceleration task. Both paths use position control, but
they obtain their targets differently. See [flight task selection][flight-tasks]
and [Navigator's mode selection][navigator].

With the default pilot-override configuration, moving the sticks in multicopter
Hold can switch back to Position, or Altitude if position is unavailable. That
is a mode change governed by takeover policy. See the
[multicopter Hold behavior][mc-hold].

### Example: fixed-wing Cruise and Hold

In Cruise, centering the sticks lets the aircraft maintain a straight ground
course and altitude. Selecting Hold tells it to loiter around a location instead.
In both cases it continues flying. See the [Cruise description][fw-cruise] and
[fixed-wing Hold behavior][fw-hold].

Cruise uses the same internal `NAVIGATION_STATE_POSCTL` identifier as multicopter
Position. The vehicle-aware [mode-name helper][mode-names] calls it "Cruise" for
fixed-wing flight; some tools still call it "Position." Hold uses
`NAVIGATION_STATE_AUTO_LOITER` for both vehicle types. When reading a mode name or
log, interpret the navigation state together with the vehicle type.

## VTOL has a separate flight configuration

A VTOL aircraft has a flight mode and a multicopter/fixed-wing transition state
at the same time. For example, it can be in Mission while hovering, transitioning,
or flying as a fixed-wing aircraft.

[`VtolVehicleStatus.vehicle_vtol_state`][vtol-status] distinguishes multicopter
flight, transition to fixed-wing, fixed-wing flight, and transition to multicopter.
Commander uses this to update `VehicleStatus.vehicle_type` and its transition
flags. In this revision, `vehicle_type` reports rotary-wing during transitions;
the transition flags carry the additional information. See
[`Commander::vtolStatusUpdate()`][commander-vtol].

VTOL generally follows the behavior of its current flight configuration, with
additional handling for takeoff, landing, and Return. During a transition,
control paths also account for the transition itself: the multicopter flight
mode manager can select a transition task, and fixed-wing mode management can
remain active. See the [VTOL overview][vtol-overview],
[flight task selection][flight-tasks], and [fixed-wing mode management][fw-mode-manager].

### Example: one VTOL mission across a transition

Consider a mission containing a VTOL takeoff, fixed-wing waypoints, and a VTOL
landing. Table 4 shows how the active flight mode can stay the same throughout
these phases.

*Table 4. Flight mode and VTOL state during an example mission.*

| Mission phase | Active flight mode | VTOL state |
| --- | --- | --- |
| Vertical ascent | Mission | Multicopter |
| Transition for forward flight | Mission | Transition to fixed-wing |
| Fly the waypoint route | Mission | Fixed-wing |
| Transition for vertical landing | Mission | Transition to multicopter |
| Vertical descent and landing | Mission | Multicopter |

The mission items advance within Mission mode. A VTOL transition command changes
the flight configuration without necessarily changing `nav_state`. The
[VTOL mission description][vtol-mission] covers these commands, and
[VTOL attitude control][vtol-control] handles transition requests and the
transition state. A feature that changes transition behavior must consider the
flight configuration as well as the active mode.

## Mode requirements and failsafe changes

Modes depend on usable estimates and inputs. Altitude needs a local altitude
estimate; Position adds a horizontal position requirement. Mission needs a valid
mission and the required navigation estimates. A position estimate can come
from sources other than satellite navigation, so these requirements describe
estimator outputs rather than a particular sensor.

[`getModeRequirements()`][mode-requirements] records requirements as bitmasks,
with one bit per navigation state. Health and arming checks use them to determine
which modes can run and which can be armed. A mode can also explicitly prevent
arming: Return and Land are examples.

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

This explains why `nav_state_user_intention` and `nav_state` can differ. The
vehicle may be executing a fallback while still recording the requested mode.
When diagnosing behavior, inspect both fields and the reported health and
failsafe state. A failsafe can also activate the same Return or Land behavior
that an operator can request; its origin and the active mode are separate facts.

## Basic Offboard behavior

Offboard delegates setpoint generation to an external system through supported
MAVLink messages or ROS 2 topics. The requested control level determines which
PX4 controllers remain active. For example, multicopter position setpoints retain
position control and the inner control loops; attitude setpoints retain attitude
and rate control while the external system determines the desired attitude.
See the [Offboard branch of the controller mapping][control-mode].

PX4 requires a stream of recent Offboard control messages before arming in
Offboard or entering the mode while armed. It also checks the estimates needed
for the requested control level. If that stream is lost, `COM_OF_LOSS_T` determines the loss timeout and
`COM_OBL_RC_ACT` selects the configured response, subject to failsafe handling.
The [Offboard health check][offboard-check] and
[Offboard documentation][offboard-docs] describe this behavior.

MAVLink setpoint messages carry both the target and evidence that the controller
is still sending. With ROS 2, `OffboardControlMode` supplies the control-level
selection and keepalive, while separate topics carry setpoints. A live keepalive
alone does not establish that a setpoint publisher is still updating. Supported
setpoint types and coordinate frames depend on the interface and vehicle, so a
multicopter Offboard example should not be assumed to apply to fixed-wing flight.

## Finding the right place for a feature

Start by identifying which part of the behavior changes. Input interpretation
or motion shaping usually leads to flight tasks or vehicle-specific mode
management. A change to automatic route behavior leads to Navigator. A new
requirement or fallback policy leads to Commander and its health or failsafe
checks. Controller response and actuator mapping belong farther down the
control path.

For example, changing how a multicopter brakes when Position sticks are released
starts with the selected manual flight task. Changing the destination used by
Return starts with Navigator's Return implementation. Changing when either mode
is allowed to run starts with mode requirements and health checks. These are
different feature locations even though all three affect what a pilot experiences
as a flight mode.

[revision]: https://github.com/PX4/PX4-Autopilot/commit/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332
[vehicle-status]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/msg/versioned/VehicleStatus.msg
[vehicle-control-mode]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/msg/VehicleControlMode.msg
[control-mode]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/ModeUtil/control_mode.cpp#L46-L163
[setpoint-types]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/ModeUtil/setpoint_types.cpp#L46-L166
[commander-state]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/Commander.cpp#L2547-L2599
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
