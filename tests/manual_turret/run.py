#!/usr/bin/env python3
"""JDK-only manual aiming and robot-heading correction regression."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='manual-turret-test-') as out:
    subprocess.run(['javac', '-d', out,
                    *[str(ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/methods' / (name + '.java'))
                      for name in ('ManualTurretAimControl', 'TurretHeadingCalibration', 'CoordinateAim', 'CoordinateTurretControl')],
                    str(Path(__file__).with_name('ManualTurretAimRegression.java'))], check=True)
    subprocess.run(['java', '-cp', out,
                    'org.firstinspires.ftc.teamcode.methods.ManualTurretAimRegression'], check=True)
