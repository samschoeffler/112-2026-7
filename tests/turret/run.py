#!/usr/bin/env python3
"""Run turret regressions against cached FTC 12.0.0 SDK interfaces."""
import sys
sys.dont_write_bytecode = True
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ftc_test_runner import run_tests

run_tests(('Turret', 'TurretCalibration', 'LimelightVision'), Path(__file__).with_name('TurretRegression.java'), 'TurretRegression')
