#!/usr/bin/env python3
"""Run control regressions with real FTC SDK interfaces and simulated hardware.

Requires a JDK (JAVA_HOME or PATH) and cached FTC 12.0.0 dependencies from an
Android build. No robot connection, downloads, or Gradle cache writes.
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
CACHE = Path(os.environ.get('GRADLE_USER_HOME', Path.home() / '.gradle')) / 'caches/modules-2/files-2.1'
JAVA_HOME = os.environ.get('JAVA_HOME')

def java_tool(name):
    result = str(Path(JAVA_HOME) / 'bin' / name) if JAVA_HOME else shutil.which(name)
    if not result:
        raise SystemExit(f'{name} not found; set JAVA_HOME to JDK 17 or newer')
    return result

def run_tests(source_names, test_file, test_class):
    with tempfile.TemporaryDirectory(prefix='ftc-control-tests-') as temp:
        directory = Path(temp)
        jars = []
        for artifact in ('RobotCore', 'Hardware'):
            aar = next((CACHE / f'org.firstinspires.ftc/{artifact}/12.0.0').glob('*/*.aar'), None)
            if aar is None:
                raise SystemExit(f'Build the Android project first: cached {artifact} 12.0.0 is missing')
            jar = directory / f'{artifact}.jar'
            with zipfile.ZipFile(aar) as archive:
                jar.write_bytes(archive.read('classes.jar'))
            jars.append(str(jar))
        # Android types occur in SDK signatures, but these tests do not invoke Android APIs.
        sdk = Path(os.environ.get('ANDROID_HOME', Path.home() / 'Library/Android/sdk'))
        platforms = sorted((sdk / 'platforms').glob('android-*/android.jar'))
        if not platforms:
            raise SystemExit('Set ANDROID_HOME to an installed Android SDK')
        jars.append(str(platforms[-1]))
        jars += [str(p) for p in (CACHE / 'com.google.code.gson/gson').glob('*/*/*.jar')
                 if not p.name.endswith(('-sources.jar', '-javadoc.jar'))]
        classpath = os.pathsep.join(jars)
        classes = directory / 'classes'
        sources = ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/methods'
        subprocess.run([java_tool('javac'), '-cp', classpath, '-d', str(classes),
                        *[str(sources / f'{name}.java') for name in
                          source_names],
                        str(test_file)], check=True)
        subprocess.run([java_tool('java'), '-cp', str(classes) + os.pathsep + classpath,
                        'org.firstinspires.ftc.teamcode.methods.' + test_class], check=True)
