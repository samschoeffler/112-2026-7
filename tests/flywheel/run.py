#!/usr/bin/env python3
"""Run deterministic flywheel tuning simulations; JDK only, no robot or SDK."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/methods'
with tempfile.TemporaryDirectory(prefix='flywheel-tests-') as output:
    subprocess.run(['javac', '-d', output,
                    *[str(SOURCES / (name + '.java')) for name in ('FlywheelVelocityControl', 'FlywheelAutoTune', 'FlywheelPositionVelocity', 'FlywheelIntegralControl')],
                    str(Path(__file__).with_name('FlywheelAutoTuneRegression.java')),
                    str(Path(__file__).with_name('FlywheelPositionVelocityRegression.java')),
                    str(Path(__file__).with_name('FlywheelIntegralRegression.java'))], check=True)
    subprocess.run(['java', '-cp', output,
                    'org.firstinspires.ftc.teamcode.methods.FlywheelAutoTuneRegression'], check=True)
    subprocess.run(['java', '-cp', output,
                    'org.firstinspires.ftc.teamcode.methods.FlywheelPositionVelocityRegression'], check=True)
    subprocess.run(['java', '-cp', output,
                    'org.firstinspires.ftc.teamcode.methods.FlywheelIntegralRegression'], check=True)
