#!/usr/bin/env python3
"""JDK-only calibration workflow and transactional CSV regression."""
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/methods'
with tempfile.TemporaryDirectory(prefix='shooting-calibration-') as out:
    subprocess.run(['javac', '-d', out,
                    str(SOURCES / 'ShootingCalibrationSession.java'),
                    str(SOURCES / 'ShootingCalibrationCsv.java'),
                    str(Path(__file__).with_name('ShootingCalibrationRegression.java'))], check=True)
    subprocess.run(['java', '-cp', out,
                    'org.firstinspires.ftc.teamcode.methods.ShootingCalibrationRegression'], check=True)

sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT / 'tests'))
from ftc_test_runner import run_tests
run_tests(('Launcher', 'Transfer', 'Intake', 'BatteryVoltage', 'FlywheelVelocityControl',
           'FlywheelIntegralControl', 'SynchronizedFeed', 'ShootingCoordinator', 'CalibrationVolleyControl'),
          Path(__file__).with_name('CalibrationVolleyRegression.java'), 'CalibrationVolleyRegression')
