# Coordinate aiming: Stage 1

Select **Coordinate Aim - Stage 1** in the Tests group. This OpMode only drives
the chassis and displays geometry. Turret motor power is held at zero; it does
not reset the turret encoder, initialize vision, or operate the shooter/intake.
Existing autonomous and turret control code is unchanged.

## Setup and controls

1. Place the robot at the test starting pose **(56, 8), heading 90 degrees**
   before pressing INIT. Coordinates are inches, with the user's bottom-left
   field origin. This test pose is independent of `AutoPaths.Red2Tip.start`.
2. Select the OpMode and press INIT. Expect the readings below before moving.
3. Press START. Hold gamepad 1 **A button** while using the sticks to drive
   with MainTeleOp's field-centric convention at 75% power. Releasing A
   requests zero drive power. There is no automatic position hold.
4. Do not reinitialize at a different location: INIT assumes the starting pose.
   Return to the start before restarting the test.

The target is **(57.4, 82.3)**. Turret pivot offsets are X = -2.25 inches, Y = 0; see [offset setup](coordinate-aim-offsets.md).
`Relative aim` is positive LEFT/counterclockwise, negative RIGHT/clockwise,
with zero straight ahead. Field bearings are measured from +X.

## Expected checks

| Robot pose (X, Y, heading degrees) | Field bearing | Relative aim |
| --- | --- | --- |
| (56, 8, 90), initial pose | 88.92 degrees | -1.08 degrees (right) |
| (56, 8, 0), rotate in place | 88.92 degrees | +88.92 degrees (left) |
| (56, 8, 180), rotate in place | 88.92 degrees | -91.08 degrees (right) |
| (57.4, 62.3, 90) | 90 degrees | 0 degrees (forward) |
| (37.4, 82.3, 90) | 0 degrees | -90 degrees (right) |
| (77.4, 82.3, 90) | 180 degrees | +90 degrees (left) |

Use the actual reported pose when comparing: imperfect placement or translation
during rotation changes the expected values. Start with the first three checks,
then translate to accessible positions. Cross heading zero slowly: the relative
aim should remain continuous there unless the target crosses directly behind
the robot. A jump between +180 and -180 behind the robot is normal wrapping.
At the target itself, direction is undefined and telemetry reports that.

`Provisional encoder target` follows the existing controller's sign convention
(`-HEADING_COMPENSATION_SIGN * relative aim`). It is not a motor command, and its
physical sign still needs verification before Stage 2. Mechanical reachability
is not evaluated in this stage.

## Report back before Stage 2

Record robot X/Y/heading, field bearing, relative aim, and provisional encoder
target at the initial pose, after rotation, and after translation. Note whether
LEFT/RIGHT agrees with the real target direction, whether pose matches physical
placement, and whether the turret remained unpowered.

Desktop validation: `CoordinateAimRegression.java` covers cardinal directions,
heading wraparound, the initial pose, invalid inputs, and coincident target.
The new OpMode also compiles with cached FTC 12.0.0 / Pedro 3.0.1 dependencies.
An Android APK build and physical checks are still required on the robot.
