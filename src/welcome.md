# PX4 autopilot notes

These notes explain PX4 for software developers adding features to the autopilot.
They connect vehicle behavior to the source code that implements it.

Start with [flight modes](concepts/flight_modes.md) to understand how PX4
interprets commands and controls multicopters, fixed-wing aircraft, and vertical
takeoff and landing (VTOL) aircraft.

Continue with [VTOL flight and transitions](concepts/vtol_flight_and_transitions.md)
to follow how navigation modes, transition supervision, controllers, and motor
allocation work together.

The [skid-to-turn quad tailsitter](airframes/skid_to_turn_quad_tailsitter.md)
chapter describes a proposed motor-only vehicle and the changes needed to
control its velocity vector during forward flight, using a shared rotational
controller across hover, transition, and cruise.

The notes follow PX4's development branch, `main`. Each chapter identifies the
revision used to verify its explanations and links to that revision of the source.
