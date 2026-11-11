# Coordinate turret pivot offsets

Set `TURRET_OFFSET_X_INCHES` and `TURRET_OFFSET_Y_INCHES` in
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/methods/CoordinateAim.java`.
The configured displacement is X = -2.25 inches and Y = 0.0 inches: the turret
is 2.25 inches behind the robot center of rotation (the localization pose origin),
with no sideways displacement.

- **+X is forward**, toward the front of the robot; negative X is backward.
- **+Y is left**, looking toward the front of the robot; negative Y is right.
- Both values are **inches**, fixed relative to the chassis regardless of turret angle.

These are robot-relative directions, not fixed field axes. At heading zero,
forward aligns with field +X and left with field +Y. At heading 90 degrees,
forward aligns with field +Y and left with field -X.

For robot heading h, the field pivot is:

```text
pivotX = robotX + offsetX * cos(h) - offsetY * sin(h)
pivotY = robotY + offsetX * sin(h) + offsetY * cos(h)
```

Bearing and distance are calculated from this pivot to the target. Relative
bearing remains positive left/counterclockwise and negative right/clockwise.
The shared settings apply to all callers of CoordinateAim, including coordinate
aim test modes. Stage 3 feedforward follows the corrected target angle as it changes.
Test telemetry shows the configured offsets. Existing zero-offset expected
readings apply only while both offsets are zero.
