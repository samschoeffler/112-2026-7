# Physical 90-degree encoder measurement

Select **Turret Encoder Scale - 90 Degrees** under Tests. This is a passive test:
turret motor power is always zero, its zero-power behavior is temporarily FLOAT,
and no drivetrain or other mechanism is initialized. No targeting, encoder reset,
or automatic configuration changes occur. Robot field pose does not matter.

1. Keep the robot stationary. INIT, then START.
2. Gently position the unpowered turret straight along the robot's front. If the
   mechanism cannot be moved by hand without forcing it, stop and report that;
   this test does not provide a powered jog.
3. Let it stop, then press **GAMEPAD 2 Y** to save the forward encoder reference.
4. Physically move the turret exactly **90 degrees LEFT**, viewed from behind
   the robot. Its aiming axis should be perpendicular to the robot's forward
   axis. Use a square, protractor, or physical reference lines; do not use the
   displayed turret angle to decide where 90 degrees is.
5. Let it stop, then press **GAMEPAD 2 X**. Record **SAVED signed delta ticks**,
   **Measured ticks / turret degree**, and **Scale multiplier vs current config**.
6. Return physically forward and press **GAMEPAD 2 Y** again. Repeat the same
   measurement at **90 degrees RIGHT**. Take a second sample per side if practical.
   Always start from a fresh physical forward reference; do not sample 180 degrees
   of travel by moving from one side directly to the other without resetting zero.
7. STOP when finished. The previous zero-power behavior is restored.

Current assumptions predict **256.333 ticks in magnitude per 90 degrees**:
384.5 motor ticks/revolution times (40/15) external ratio divided by four.
Signs may differ by direction; the magnitude determines scale. Based on the
earlier reported sign calibration, left should increase raw ticks and right
should decrease them, with the motor direction set to FORWARD as in Stage 2.

The implied motor ticks/revolution is an estimate assuming that the external
ratio is correct and there are no additional reductions. This measurement alone
cannot identify which mechanical/encoder assumption is wrong. Large differences
between repeated samples or between sides call for checking alignment, backlash,
or slipping before applying a new scale.

Report signed raw deltas for left and right and how you measured the physical
90-degree positions. We will use those results before changing any constants.
