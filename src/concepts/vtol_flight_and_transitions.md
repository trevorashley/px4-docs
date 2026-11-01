# VTOL flight and transitions

A vertical takeoff and landing (VTOL) aircraft combines a flight mode with a
flight configuration. Mission, for example, can remain active while the
aircraft climbs vertically, transitions, and follows a route in forward
flight. Transition changes the motion references and the control outputs
used to fly the aircraft, while the active flight mode determines its task.

This chapter explains how PX4's subsystems coordinate those changes. It uses
the stock quad tailsitter to make the control path concrete, then relates it
to the proposed [skid-to-turn vehicle](../airframes/skid_to_turn_quad_tailsitter.md).
The shared geometric controller described there is a proposed replacement for
parts of this path and has yet to be implemented.

Source revision: PX4-Autopilot `main`, checked on 2026-11-01 at
[`53b193a6fb1c7aec34fb7334a4ca91d4b9b20332`][revision]. Links are pinned to that
revision. Transition details below are specific to the tailsitter unless
stated otherwise.

## Flight mode and VTOL configuration

`VehicleStatus.nav_state` identifies the active flight mode. Its companion,
`nav_state_user_intention`, records the requested mode, which can differ from
the active mode during a failsafe. These fields describe the selected behavior,
such as pilot-assisted flight, a mission, or external control. See
[flight modes](flight_modes.md) for mode selection and request arbitration.

[`VtolVehicleStatus.vehicle_vtol_state`][vtol-status] records the flight
configuration separately. Table 1 shows its four operating states and how
Commander represents them in `VehicleStatus`.

*Table 1. VTOL states and the corresponding vehicle-status fields.*

| VTOL state | Meaning | `vehicle_type` | Transition flags |
| --- | --- | --- | --- |
| `VEHICLE_VTOL_STATE_MC` (`3`) | Multicopter flight, including hover. | Rotary-wing | Both false |
| `VEHICLE_VTOL_STATE_TRANSITION_TO_FW` (`1`) | Transition toward forward flight. | Rotary-wing | `in_transition_mode` and `in_transition_to_fw` true |
| `VEHICLE_VTOL_STATE_FW` (`4`) | Fixed-wing flight configuration. | Fixed-wing | Both false |
| `VEHICLE_VTOL_STATE_TRANSITION_TO_MC` (`2`) | Transition toward multicopter flight. | Rotary-wing | `in_transition_mode` true, `in_transition_to_fw` false |

The message also defines an undefined value (`0`). The table describes the
operating states rather than that placeholder. In this revision,
[`Commander::vtolStatusUpdate()`][commander-vtol] reports rotary-wing type
during both transitions. Code that needs to identify hover must therefore
inspect the transition flags as well as `vehicle_type`.

The selected VTOL implementation is another separate concept. `VT_TYPE`
selects tailsitter (`0`), tiltrotor (`1`), or standard VTOL (`2`). A tailsitter
rotates the aircraft into forward flight, a tiltrotor changes rotor
orientation, and a standard VTOL uses separate lift and forward-flight
propulsion. The [VTOL module constructor][vtol-constructor] selects the
corresponding implementation. The [VTOL airframe overview][vtol-types]
describes these physical configurations. The quad-tailsitter preset uses
`VT_TYPE=0`.

## Flight modes available to a VTOL

VTOL aircraft use the multicopter and fixed-wing mode families, with behavior
depending on flight configuration. Table 2 covers the primary modes relevant
to this design. It describes stock PX4 behavior, which must be revisited where
the proposed skid-to-turn controller changes how commands are interpreted.

*Table 2. Primary flight modes and their significance for VTOL operation.*

| Mode | Behavior and VTOL considerations |
| --- | --- |
| Manual | Uses attitude stabilization in multicopter configuration. In fixed-wing configuration, pilot inputs produce thrust and torque commands without attitude or rate stabilization. |
| Stabilized | Tracks pilot attitude requests. The pilot remains responsible for altitude and thrust. |
| Acro | Tracks pilot angular-rate requests. This requires a rate-command path even if automatic flight uses a combined geometric attitude-and-rate law. |
| Altitude | Adds vertical-motion control. The stock fixed-wing interpretation uses bank commands for lateral maneuvering. |
| Position / Cruise | Position control in multicopter flight can brake and hover. Fixed-wing Cruise maintains forward motion along a course and altitude. Both use `NAVIGATION_STATE_POSCTL`. |
| Hold | Hovers at a location in multicopter flight and follows a loiter pattern in fixed-wing flight. Selecting Hold alone does not request a transition to hover. |
| Mission | Executes mission items, including VTOL takeoff, transition, and landing items. The mission can span all four VTOL states. |
| Return | Selects a return route and landing behavior according to the return configuration. It must account for the current flight configuration and any required back transition. |
| Takeoff / VTOL Takeoff | Ordinary Takeoff uses `NAVIGATION_STATE_AUTO_TAKEOFF`. The distinct `NAVIGATION_STATE_AUTO_VTOL_TAKEOFF` behavior includes vertical ascent, heading alignment, transition, and establishment of a forward-flight loiter. |
| Land | Normally transitions a VTOL in forward flight back to multicopter configuration for vertical landing. `NAV_FORCE_VT` affects the configured landing behavior. |
| Offboard | Accepts external setpoints at the supported control level. Position, velocity, attitude, rate, and direct actuator interfaces do not all use the same controllers. |

The [control-mode mapping][control-mode] determines which controller levels
are enabled. The [VTOL mode overview][vtol-modes] and its
[Mission][vtol-mission], [Return][vtol-return], and [Land][vtol-land]
descriptions explain the mode-specific behavior. Return has several route
strategies, so selecting it does not universally mean transitioning to
fixed-wing flight. Likewise, a VTOL takeoff mission item is executed inside
Mission and does not require changing `nav_state` to the standalone VTOL
Takeoff mode. The latter has its own [Navigator implementation][vtol-takeoff].

The mode enumeration also includes specialized behaviors, such as Guided
Course, Orbit, and Precision Land. An enum value alone does not establish that
a particular vehicle build can enter or execute that mode. Inspect
`VehicleStatus.valid_nav_states_mask` and `can_set_nav_states_mask` for the
advertised mode set, and use the [mode requirements][mode-requirements] and
health checks to understand whether it can run with the available estimates
and inputs. The [status message][vehicle-status] defines those fields.

## How the subsystems cooperate

The VTOL module coordinates the control paths, while other modules decide
the flight task, estimate motion, generate references, and allocate actuator
commands. Table 3 identifies the main responsibilities.

*Table 3. Subsystems involved in VTOL flight and transition.*

| Subsystem | Responsibility |
| --- | --- |
| Commander | Selects the active navigation state, applies mode and failsafe policy, and publishes control flags and vehicle type. |
| Navigator | Generates automatic navigation targets and mission work items, including transition requests. |
| Estimation and airspeed selection | Supply attitude, angular velocity, position, velocity, wind, and airspeed inputs used by controllers and transition checks. |
| `flight_mode_manager` and `mc_pos_control` | Generate and track multicopter motion references. A transition flight task can supply references during transition. |
| `fw_mode_manager` and `fw_lat_lon_control` | Generate fixed-wing guidance and attitude/thrust references. Parts of this path also run during transition. |
| `vtol_att_control` and its `Tailsitter` implementation | Manage VTOL state, construct or select attitude references, check transition conditions, and route thrust and torque outputs. |
| Multicopter and fixed-wing attitude/rate controllers | Turn attitude or rate references into torque commands for their respective control paths. |
| `control_allocator` and actuator output | Convert the selected torque and thrust demands into individual motor and, where present, control-surface commands. |

The [VTOL startup script][vtol-startup] starts both controller families.
Transition generally changes execution conditions and which outputs are used,
so it does not simply stop one complete stack and start another. For example,
the multicopter flight mode manager disables its flight tasks in fixed-wing
configuration and selects `FlightTaskTransition` during transition when
altitude control is enabled. That task moves the vertical-velocity reference
toward zero. See [task selection][flight-task-selection] and the
[transition task][transition-task].

The topic names expose two boundaries. Outer controllers publish
`mc_virtual_attitude_setpoint` and `fw_virtual_attitude_setpoint`. The VTOL
module selects or constructs the active `vehicle_attitude_setpoint`.
Separately, rate controllers publish virtual multicopter and fixed-wing
torque/thrust setpoints, and the VTOL module routes the selected output to
the allocator. The word *virtual* identifies an input branch of that routing,
rather than a separate simulated aircraft. The
[VTOL reference and output selection][vtol-routing] shows both boundaries.

For the stock motor-only tailsitter, Table 4 identifies the path used for
motor control. It does not imply that the other modules have stopped running.

*Table 4. Active attitude and motor-command paths for the stock tailsitter.*

| Flight configuration | Active attitude reference | Rotational control used for the motors |
| --- | --- | --- |
| Multicopter | Multicopter reference, forwarded by the VTOL module. | Multicopter attitude and rate control. |
| Either transition | Transition attitude constructed by `Tailsitter`, with multicopter thrust input and transition blending. | Multicopter attitude and rate control. |
| Fixed-wing | Fixed-wing reference, with handover handling in the VTOL module. | Fixed-wing attitude and rate control, routed to the motors through enabled differential-thrust axes. |

The multicopter attitude controller explicitly
[enables control during hover and tailsitter transitions][mc-attitude-gate].
The tailsitter's [`fill_actuator_outputs()`][tailsitter-outputs] selects motor
torque and thrust. In forward flight it maps fixed-wing forward thrust onto
the motor thrust axis and applies the differential-thrust axis settings.
The quad-tailsitter preset has no control surfaces and enables all three
differential-thrust axes with `VT_FW_DIFTHR_EN=7`.

## From a request to a transition

A transition request specifies the desired flight configuration. It can come
from several sources:

- A radio-controller transition switch produces an
  `ACTION_VTOL_TRANSITION_TO_FIXEDWING` or
  `ACTION_VTOL_TRANSITION_TO_MULTICOPTER` action request.
- A ground station or companion computer can send
  `VEHICLE_CMD_DO_VTOL_TRANSITION`.
- Navigator can issue the transition command while executing a mission,
  VTOL takeoff, or landing sequence.

The [manual-control switch handler][transition-switch] and
[VTOL request handlers][transition-requests] show the input paths. For the
transition command, `param1=4` requests fixed-wing configuration and
`param1=3` requests multicopter configuration. These values identify VTOL
states, not navigation states.

The VTOL module stores a requested configuration in `_transition_command`.
The tailsitter state machine compares that request with its current phase,
enters a transition phase, and resets transition timing when appropriate.
The aircraft then follows transition references until a completion condition,
an abort, or a failure changes the state. An accepted command acknowledgment
therefore means the request was accepted, rather than that transition has
finished. Navigator explicitly [checks the resulting vehicle state][mission-transition-completion]
before completing a transition mission item.

Mode policy can affect the request. In this revision, the command handler
temporarily rejects requests for fixed-wing configuration in ordinary Takeoff,
Land, Return, and Orbit. Entering Return during a front transition requests
multicopter configuration. These checks are in the
[VTOL request-handling code][transition-requests], and the RC action handler
is a separate input path. They should not be generalized into a single
universal mode-switch rule.

## A stock tailsitter transition in detail

For this aircraft, transition rotates the entire airframe. The propellers
remain fixed to it, so the change in orientation redirects propulsive force
while increasing or decreasing the contribution from aerodynamic lift.
Control must manage the aircraft's motion during that change as well as
decide when the next flight configuration is ready.

### Transition to fixed-wing flight

When fixed-wing flight is requested from multicopter flight,
[`Tailsitter::update_vtol_state()`][tailsitter-state] enters
`TRANSITION_FRONT_P1`. This internal name belongs to the tailsitter state
machine and is reported externally as transition to fixed-wing flight.

The [transition-reference generator][tailsitter-reference] starts from a
multicopter attitude reference and constructs a quaternion rotation toward
the cruise attitude. `VT_F_TRANS_DUR` sets the nominal rotation rate, and
`FW_PSP_OFF` affects the target tilt. The code requires a recent multicopter
attitude setpoint, so changing controller publication behavior can also stop
transition-reference updates. Multicopter rotational control tracks this
reference while the VTOL module routes its motor commands.

The airborne front-transition completion rule is specific to the tailsitter:

- Measured pitch in the hover body-frame convention must reach approximately
  -60 degrees (`-1.05 rad`) or farther toward forward flight.
- If the VTOL module has a finite usable calibrated airspeed, that airspeed
  must also reach `VT_ARSP_TRANS`.
- Without that usable airspeed input, the pitch condition alone completes
  the airborne transition.

These conditions come from the [tailsitter completion override][tailsitter-completion]
and its [angle constants][tailsitter-constants]. The shared wrapper also
allows completion while disarmed or landed. The generic VTOL base class has
minimum-time and open-loop-time rules, but the tailsitter overrides its
airborne completion test. `VT_F_TRANS_DUR` therefore shapes the rotation
reference and does not promise that the state changes after exactly that
duration. See the [base completion wrapper][base-completion].

Usable airspeed here means the input actually accepted by
[`vtol_att_control`][airspeed-input], which checks the configured use of
airspeed and the selected sensor source. The presence of an airspeed field
elsewhere in a log does not establish that this transition check used it.

On completion, the module publishes fixed-wing VTOL state. Commander updates
vehicle type, controller execution conditions change, and the VTOL module
selects fixed-wing references and motor commands. The
[handover logic][tecs-handover] checks readiness of the total energy control
system (TECS) where altitude control requires it. The tailsitter output path
also retains the last multicopter thrust and suppresses torque for the first
50 ms after the switch. These are separate parts of handing control to the
forward-flight path.

### Transition to multicopter flight

A multicopter request from fixed-wing flight enters `TRANSITION_BACK`.
The tailsitter generates a rotation toward the upright attitude and uses
multicopter rotational control during the maneuver. It blends the previous
fixed-wing thrust into the multicopter thrust reference over 0.5 s, with
additional short output-side thrust continuity handling. See the
[transition reference][tailsitter-reference],
[thrust blending][tailsitter-completion], and [motor routing][tailsitter-outputs].

The [state machine][tailsitter-state] declares the back transition complete
when hover-frame pitch reaches approximately -15 degrees (`-0.26 rad`) or
when the elapsed time exceeds `VT_B_TRANS_DUR`. The same parameter also
sets the nominal back-transition rotation rate. This condition does not
require zero horizontal speed, so entering multicopter state does not mean
that the aircraft has already stopped moving. Hover motion control may still
need to brake and settle onto its target.

The quad-tailsitter [simulation preset][quad-preset] sets defaults of
`VT_F_TRANS_DUR=1.5 s`, `VT_B_TRANS_DUR=5 s`, and `VT_ARSP_TRANS=15 m/s`.
These are model-specific settings, not established values for the proposed
vehicle. Shared parameter descriptions also cover other VTOL implementations,
so their exact effect must be checked in the selected implementation.

## Aborts and emergency recovery

A changed request can interrupt a transition. The tailsitter state machine
returns to multicopter state if a front transition loses its fixed-wing
request. If fixed-wing flight is requested during a back transition, it
returns to fixed-wing state. These state changes are defined in
[`update_vtol_state()`][tailsitter-state] and do not imply that the physical
aircraft instantly reaches the corresponding steady flight condition.

A *quad-chute* is an emergency change to multicopter control following a
fixed-wing or transition failure. Depending on enabled checks and operating
conditions, triggers include transition timeout, altitude loss, and attitude
limits. The [failure-checking code][quadchute-checks] evaluates these
conditions, and the [VTOL failure handler][quadchute-handler] records
`fixed_wing_system_failure`. The
tailsitter responds by selecting multicopter state directly, bypassing its
normal back-transition sequence.

The subsequent flight-mode response is a separate Commander decision.
`COM_QC_ACT` maps to warning only, Return, Land, or Hold. In this revision,
the dedicated quad-chute failsafe action is evaluated for intended Mission,
Hold, Takeoff, and VTOL Takeoff modes. Other active failures can also affect
the final action. See the [action mapping][quadchute-actions] and
[failsafe condition][quadchute-failsafe]. A change in VTOL state and a change
in `nav_state` should therefore be diagnosed separately.

## Implications for the skid-to-turn vehicle

The proposed vehicle can retain the four VTOL states while using the
[shared geometric rotational controller](../airframes/skid_to_turn_quad_tailsitter.md#shared-rotational-control-architecture)
in every phase. Hover, transition, and cruise would select different motion
references, while one rotational controller tracks their orientation and
angular motion. Fixed-wing state would continue to identify the cruise
configuration without requiring the stock bank-to-turn control law.

Several interfaces must change together:

1. Transition reference generation must supply smooth attitude, angular
   velocity, angular acceleration, and thrust references in the agreed body
   frame. Reusing the state machine does not by itself provide those signals.
2. VTOL supervision must run independently of the virtual torque publications
   currently used to [schedule the module][vtol-scheduling]. The new actuator
   interface must own the motor torque/thrust stream without competing with
   the stock VTOL output path.
3. Completion and readiness checks must describe this vehicle's ability to
   sustain the requested motion. Cruise readiness must replace dependencies
   on the stock TECS/controller handover, and the stock pitch, airspeed, and
   timeout rules need evaluation for this airframe.
4. Emergency recovery must generate an achievable reference for the shared
   controller. Setting multicopter state alone does not rotate the vehicle,
   remove its forward speed, or establish enough upward force to arrest a
   descent. Checks that depend on TECS altitude references also need an
   equivalent input from the new motion-control path.

For example, a mission can still request vertical takeoff, transition,
waypoint flight, and vertical landing. During waypoint flight, the new cruise
outer controller would turn navigation objectives into velocity-vector
commands and the attitude/thrust needed for skid-to-turn motion. During
transition, the supervisor would select the transition trajectory. The
navigation state can remain Mission throughout this sequence, even though
the reference source and physical flight regime change.

Mode semantics also remain part of the implementation. Hold in cruise needs
a feasible moving path or an explicit transition policy. Manual, Acro, and
Offboard need defined command interpretation at their respective control
levels. A shared attitude controller alone does not supply those behaviors.
The [design chapter's integration scope](../airframes/skid_to_turn_quad_tailsitter.md#px4-integration-scope)
describes the proposed component changes and evaluation stages.

## Tracing a transition in the software

To understand a transition, follow the request, state, reference, and actuator
paths together:

- Inspect `vehicle_command` or `action_request` for the requested direction,
  then distinguish command acknowledgment from completed VTOL state.
- Read `vtol_vehicle_status` alongside `vehicle_status.nav_state`,
  `nav_state_user_intention`, `vehicle_type`, and the transition flags.
- Compare the virtual attitude references with `vehicle_attitude_setpoint`
  and the measured attitude. Interpret their frames using the
  [tailsitter frame convention](../airframes/skid_to_turn_quad_tailsitter.md#reference-generation-and-body-frames).
- Follow the virtual torque/thrust inputs to the selected allocator inputs
  and motor outputs. Include airspeed selection, TECS readiness where used,
  and failure events when the state changes unexpectedly.

For a transition mission item, the [completion check][mission-transition-completion]
provides a useful final boundary. Navigator waits for the resulting flight
configuration, while the VTOL subsystem and controllers perform the maneuver.

[revision]: https://github.com/PX4/PX4-Autopilot/commit/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332
[vehicle-status]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/msg/versioned/VehicleStatus.msg
[vtol-status]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/msg/versioned/VtolVehicleStatus.msg
[commander-vtol]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/Commander.cpp#L2405-L2435
[vtol-constructor]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/vtol_att_control/vtol_att_control_main.cpp#L59-L80
[control-mode]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/ModeUtil/control_mode.cpp#L46-L144
[vtol-modes]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/docs/en/flight_modes_vtol/index.md
[vtol-mission]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/docs/en/flight_modes_vtol/mission.md
[vtol-return]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/docs/en/flight_modes_vtol/return.md
[vtol-land]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/docs/en/flight_modes_vtol/land.md
[vtol-takeoff]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/navigator/vtol_takeoff.cpp
[mode-requirements]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/ModeUtil/mode_requirements.cpp
[vtol-startup]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/ROMFS/px4fmu_common/init.d/rc.vtol_apps
[flight-task-selection]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/flight_mode_manager/FlightModeManager.cpp#L139-L153
[transition-task]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/flight_mode_manager/tasks/Transition/FlightTaskTransition.cpp#L82-L150
[vtol-routing]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/vtol_att_control/vtol_att_control_main.cpp#L410-L469
[mc-attitude-gate]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/mc_att_control/mc_att_control_main.cpp#L303-L308
[tailsitter-outputs]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/vtol_att_control/tailsitter.cpp#L303-L375
[transition-switch]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/manual_control/ManualControl.cpp#L288-L294
[transition-requests]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/vtol_att_control/vtol_att_control_main.cpp#L109-L191
[mission-transition-completion]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/navigator/mission_block.cpp#L120-L134
[tailsitter-state]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/vtol_att_control/tailsitter.cpp#L101-L195
[tailsitter-reference]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/vtol_att_control/tailsitter.cpp#L197-L284
[tailsitter-completion]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/vtol_att_control/tailsitter.cpp#L379-L407
[tailsitter-constants]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/vtol_att_control/tailsitter.h#L51-L58
[base-completion]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/vtol_att_control/vtol_type.cpp#L171-L201
[airspeed-input]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/vtol_att_control/vtol_att_control_main.cpp#L382-L396
[tecs-handover]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/vtol_att_control/vtol_type.cpp#L111-L153
[quad-preset]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/ROMFS/px4fmu_common/init.d-posix/airframes/4018_gz_quadtailsitter
[quadchute-checks]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/vtol_att_control/vtol_type.cpp#L211-L400
[quadchute-actions]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/failsafe/failsafe.cpp#L235-L262
[quadchute-failsafe]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/commander/failsafe/failsafe.cpp#L620-L626
[vtol-scheduling]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/vtol_att_control/vtol_att_control_main.cpp#L264-L365
[vtol-types]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/docs/en/frames_vtol/index.md
[quadchute-handler]: https://github.com/PX4/PX4-Autopilot/blob/53b193a6fb1c7aec34fb7334a4ca91d4b9b20332/src/modules/vtol_att_control/vtol_att_control_main.cpp#L194-L244
