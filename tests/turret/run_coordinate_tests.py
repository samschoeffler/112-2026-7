#!/usr/bin/env python3
"""Run coordinate aiming math/control checks with a JDK; no FTC SDK needed."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/methods'

with tempfile.TemporaryDirectory(prefix='coordinate-tests-') as output:
    for name in ('CoordinateAim', 'CoordinateTurretControl', 'CoordinateMotionFeedforward'):
        subprocess.run(['javac', '-cp', output, '-d', output, str(SOURCES / (name + '.java')),
                        str(Path(__file__).parent / (name + 'Regression.java'))], check=True)
        subprocess.run(['java', '-cp', output,
                        'org.firstinspires.ftc.teamcode.methods.' + name + 'Regression'], check=True)
