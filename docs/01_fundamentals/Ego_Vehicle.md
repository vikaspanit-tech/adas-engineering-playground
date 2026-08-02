# Ego Vehicle

The **ego vehicle** is the vehicle whose ADAS system is being designed, simulated, or evaluated. All driving decisions are made from its point of view.

## Why It Matters

The ego vehicle provides the reference frame for many ADAS calculations:

- Its position, speed, heading, and acceleration define vehicle state.
- Detected actors are described relative to it.
- Planning creates a path for it to follow.
- Control produces commands for it to execute.

Other road users are commonly called **actors**, **target vehicles**, or **traffic vehicles**.