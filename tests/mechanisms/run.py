#!/usr/bin/env python3
"""Run mechanisms regressions against cached FTC 12.0.0 SDK interfaces."""
import sys
sys.dont_write_bytecode = True
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ftc_test_runner import run_tests

run_tests(('Launcher', 'Intake', 'LauncherIntakeControls'), Path(__file__).with_name('MechanismRegression.java'), 'MechanismRegression')
