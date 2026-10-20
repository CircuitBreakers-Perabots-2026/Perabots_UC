"""Build a clean runtime-only competition archive with required names intact."""
import argparse
import hashlib
from pathlib import Path
import re
import zipfile

ROOT = Path(__file__).resolve().parents[1]
LOCKED = {
    'Arena.proto': '23bb0016ce182e6b71597df623da742a37c9525f13dede044a327745a4f9e82b',
    'Obstacles.proto': '9c5aee2f03eddd2698fa6b2a5212e3f21fa500027721b4f310cb9bc8508b5401',
}
BODY_HASH = '3884238c44cb0f289c58224d2a6457488a8b6f3d7e589cbc9759cb1702778234'


def validate():
    for name, expected in LOCKED.items():
        if hashlib.sha256((ROOT / 'protos' / name).read_bytes()).hexdigest() != expected:
            raise ValueError('Locked competition asset changed: ' + name)
    proto = (ROOT / 'protos/TwoWheelRobot.proto').read_text()
    body = proto[proto.index('  Robot {'):]
    if hashlib.sha256(body.encode()).hexdigest() != BODY_HASH:
        raise ValueError('Base mechanics or required LED nodes changed')
    for prohibited in ('GPS {', 'Emitter {', 'Receiver {', 'Lidar {', 'supervisor TRUE'):
        if prohibited in proto:
            raise ValueError('Prohibited robot feature: ' + prohibited)
    required = ('controllers/main_controller/main_controller.py',
                'controllers/main_controller/navigation.py', 'worlds/Arena.wbt')
    for path in required:
        if not (ROOT / path).is_file():
            raise ValueError('Missing runtime file: ' + path)


def package(team, destination):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', team):
        raise ValueError('Use letters, digits, hyphens or underscores for the team identifier')
    validate()
    destination.mkdir(parents=True, exist_ok=True)
    archive = destination / (team + '_Submission.zip')
    files = [ROOT / 'README.md', *(ROOT / 'docs').glob('*.md')]
    for folder in ('controllers/main_controller', 'protos', 'worlds'):
        files.extend(p for p in (ROOT / folder).rglob('*')
                     if p.is_file() and not any(part.startswith('.') or part == '__pycache__'
                                               for part in p.relative_to(ROOT).parts))
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as output:
        for path in sorted(files):
            output.write(path, Path(team + '_Submission') / path.relative_to(ROOT))
    return archive


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--team', default='PeraBots_UC')
    parser.add_argument('--output', type=Path, default=ROOT / 'output')
    args = parser.parse_args()
    print(package(args.team, args.output))


if __name__ == '__main__':
    main()
