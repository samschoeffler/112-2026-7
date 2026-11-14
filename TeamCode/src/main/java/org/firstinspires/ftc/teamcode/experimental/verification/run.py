#!/usr/bin/env python3
"""Audit exact working baseline, compile both namespaces, and run isolated tuning regressions."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import zipfile

EXP = Path(__file__).resolve().parents[1]
ROOT = next(p for p in EXP.parents if (p / '.git').exists())
JAVA = ROOT / 'TeamCode/src/main/java'
BASE = '5267071e36537a370943cd9891aa8d8167b77837'
CACHE = Path(os.environ.get('GRADLE_USER_HOME', Path.home() / '.gradle')) / 'caches/modules-2/files-2.1'
SDK = Path(os.environ.get('ANDROID_HOME', Path.home() / 'Library/Android/sdk'))

def audit():
    entries = subprocess.check_output(['git', 'ls-tree', '-rz', BASE], cwd=ROOT).split(b'\0')
    count = 0
    for entry in entries:
        if not entry:
            continue
        metadata, path = entry.split(b'\t', 1)
        mode, kind, oid = metadata.split()
        if kind != b'blob':
            raise AssertionError('Unexpected non-file baseline entry')
        file = ROOT / path.decode()
        data = os.readlink(file).encode() if mode == b'120000' else file.read_bytes()
        actual = hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()
        assert actual == oid.decode(), f'Baseline mismatch: {file.relative_to(ROOT)}'
        if mode != b'120000':
            assert bool(file.stat().st_mode & 0o111) == (mode == b'100755'), f'File mode mismatch: {file}'
        count += 1
    extras = subprocess.check_output(['git', 'ls-files', '--others', '--exclude-standard', '-z'], cwd=ROOT).decode().split('\0')
    prefix = str(EXP.relative_to(ROOT)) + '/'
    assert all(not p or p.startswith(prefix) for p in extras), 'Extra file outside experimental folder'
    manifest = json.loads((EXP / 'manifest.json').read_text())
    for item in manifest['original_snapshots']:
        data = (EXP / item['snapshot']).read_bytes()
        assert hashlib.sha256(data).hexdigest() == item['sha256'], 'Original recovery snapshot was altered'
    names = []
    for file in EXP.rglob('*.java'):
        text = file.read_text()
        assert re.search(r'^package org\.firstinspires\.ftc\.teamcode\.experimental(?:\.|;)', text, re.M), file
        assert not re.search(r'org\.firstinspires\.ftc\.teamcode\.(?!experimental\b)', text), f'Production dependency in {file}'
        for match in re.finditer(r'@(TeleOp|Autonomous)\(([^\n]*)\)', text):
            name = re.search(r'\bname\s*=\s*"([^"]+)"', match[2])[1]
            assert name.startswith('EXP | '), name
            assert 'group = "Experimental"' in match[2], file
            names.append(name)
    assert len(names) == len(set(names)), 'Duplicate EXP OpMode names'
    print(f'AUDIT PASS: all {count} baseline files byte-for-byte/mode identical to {BASE[:7]}; '
          f'{len(names)} unique isolated EXP OpModes; all original snapshots intact', flush=True)

def verify_java():
    with tempfile.TemporaryDirectory(prefix='isolated-tuning-checks-') as temp:
        temp = Path(temp)
        jars = []
        for index, file in enumerate(sorted(CACHE.rglob('*'), reverse=True)):
            if not file.is_file() or file.name.endswith(('-sources.jar', '-javadoc.jar')):
                continue
            if file.suffix == '.jar':
                jars.append(str(file))
            elif file.suffix == '.aar':
                with zipfile.ZipFile(file) as archive:
                    if 'classes.jar' in archive.namelist():
                        output = temp / f'dependency-{index}.jar'
                        output.write_bytes(archive.read('classes.jar'))
                        jars.append(str(output))
        platforms = sorted((SDK / 'platforms').glob('android-*/android.jar'))
        if not platforms:
            raise SystemExit('Android SDK missing; set ANDROID_HOME')
        jars.insert(0, str(platforms[-1]))
        cp = os.pathsep.join(jars)
        baseline = [p for p in JAVA.rglob('*.java') if not p.is_relative_to(EXP)]
        experimental = list(EXP.rglob('*.java'))
        subprocess.run(['javac', '-proc:none', '-cp', cp, '-d', str(temp / 'baseline'), *map(str, baseline)], check=True)
        print(f'COMPILE PASS: {len(baseline)} confirmed-working Java sources alone', flush=True)
        fixtures = []
        for file in sorted((EXP / 'verification/fixtures').glob('*.java.fixture')):
            output = temp / file.name.removesuffix('.fixture')
            output.write_bytes(file.read_bytes())
            fixtures.append(output)
        classes = temp / 'combined'
        subprocess.run(['javac', '-proc:none', '-cp', cp, '-d', str(classes),
                        *map(str, baseline + experimental + fixtures)], check=True)
        print(f'COMPILE PASS: baseline + {len(experimental)} isolated experimental sources + '
              f'{len(fixtures)} desktop regressions', flush=True)
        for file in fixtures:
            subprocess.run(['java', '-cp', str(classes) + os.pathsep + cp,
                            'org.firstinspires.ftc.teamcode.experimental.methods.' + file.stem], check=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--audit-only', action='store_true')
    args = parser.parse_args()
    audit()
    if not args.audit_only:
        verify_java()
    audit()
