# PX4 Autopilot Notes

These notes explore PX4 from the perspective of a software developer adding
features to the autopilot. They connect concepts and vehicle behavior to the
source code that implements them.

Start with [flight modes](concepts/flight_modes.md) to understand how PX4
interprets commands, selects control behavior, and handles differences between
multicopter, fixed-wing, and VTOL vehicles.

The notes target the development branch, `main`. Each chapter records the
revision it was checked against and links to that revision of the source.
