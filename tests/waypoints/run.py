#!/usr/bin/env python3
"""Waypoint logic with simulated FTC mechanisms and the cached real Pedro API."""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests'))
from ftc_test_runner import run_tests, java_tool, CACHE

run_tests(('Launcher', 'Transfer', 'Intake', 'BatteryVoltage', 'FlywheelVelocityControl',
           'FlywheelIntegralControl', 'SynchronizedFeed', 'ShootingCoordinator', 'CalibrationVolleyControl',
           'ShootingWaypoints', 'WaypointShootingControl', 'ManualTurretJog'),
          Path(__file__).with_name('WaypointShootingRegression.java'), 'WaypointShootingRegression')

pedro = next((CACHE / 'com.pedropathing/core/3.0.1').glob('*/*.jar'), None)
if pedro is None:
    raise SystemExit('Build Android first: cached Pedro core 3.0.1 is required')
sources = ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/methods'
with tempfile.TemporaryDirectory(prefix='waypoint-pedro-') as tmp:
    subprocess.run([java_tool('javac'), '-cp', str(pedro), '-d', tmp,
                    str(sources / 'ShootingWaypoints.java'), str(sources / 'WaypointDrive.java'),
                    str(sources / 'AutoPaths.java'),
                    str(Path(__file__).with_name('WaypointDriveRegression.java'))], check=True)
    subprocess.run([java_tool('java'), '-cp', tmp + os.pathsep + str(pedro),
                    'org.firstinspires.ftc.teamcode.methods.WaypointDriveRegression'], check=True)
