# Experimental tuning sandbox

All original Git-tracked files are restored exactly to commit
`5267071e36537a370943cd9891aa8d8167b77837` (`auto updtates 2 tip works`).
The only new working-tree files are inside this folder. No baseline build file,
TeleOp, autonomous routine, method, controller gain, input mapping or test is edited
to load experimental code.

## Which OpMode to run

- **MainTeleOp** and the original autonomous names are the confirmed baseline versions.
- **EXP | MainTeleOp** and other names starting **EXP |** are experimental copies.
- Experimental annotated OpModes are grouped under **Experimental**.
- Existing `@Disabled` annotations remain disabled.

The sandbox preserves the current tuning work as it stood before restoring the working
commit. It includes the flywheel, intake and transfer experiments, stored-batch shooting
coordinator, turret calibration/movement tests, experimental TeleOp/autos, and their
duplicated supporting methods. This preserves work; it does not certify those experimental
changes as working on the robot. In particular, the experimental shooting code retains
the requested removal of automatic fault lockouts.

## Isolation

All sandbox Java classes use `org.firstinspires.ftc.teamcode.experimental...`.
Every internal import/reference points to that namespace. Duplicated `methods/`,
`pedro/Constants.java`, `AutoPaths` and `OpModeStorage` prevent tuning gains, paths,
configuration and autonomous/TeleOp state from using the production copies.

Because OpModeStorage is separate, a production auto does not populate experimental
TeleOp's handoff state. Use matching EXP auto/TeleOp versions when testing a handoff.
Both environments still refer to the robot's physical configured hardware; only one
OpMode should run at a time. Existing on-robot CSV output locations are preserved.

The original entry-point names inside historical notes/snapshots should be read with
the **EXP |** prefix for sandbox use. `manifest.json` provides the exact name mapping.

## What is preserved

- `manifest.json`: baseline commit, copied source hashes, entry-point mapping and recovery locations.
- `verification/originals/`: exact original bytes of every changed or additional file
  that was restored/relocated, with `.snapshot` appended so Java does not compile it.
  This includes the tuning test code and instructions as they stood before isolation.
- `verification/fixtures/`: runnable experimental regression sources renamed
  `.java.fixture` so they do not become app source files. Their package names point
  to the experimental methods.
- A complete pre-isolation working-tree recovery archive was also saved outside the
  repository; its location is recorded in the manifest. Original snapshots in this
  folder remain available even if that temporary archive is later cleaned up.

## Verification

From the repository root:

```sh
python3 TeamCode/src/main/java/org/firstinspires/ftc/teamcode/experimental/verification/run.py
```

This audits every baseline Git blob and executable bit, checks that no extra files
remain outside the sandbox, checks namespace/OpMode isolation, verifies original
snapshot hashes, compiles baseline sources alone, compiles both namespaces together,
and runs the experimental regressions against cached FTC/Android/Pedro dependencies.
It uses temporary compiler outputs and does not modify the restored baseline files.

Use `--audit-only` for the exact-baseline/isolation audit without Java compilation.
The Java check uses cached dependencies rather than producing an Android APK.

Perform a **clean rebuild/install** in Android Studio before running the restored robot
code so stale compiled classes from the previous loose experimental files are removed.
Source identity and desktop checks do not replace physical testing on the robot.

Future tuning changes belong only in this sandbox until explicitly approved for
production. Keep the original files matching the confirmed commit.
