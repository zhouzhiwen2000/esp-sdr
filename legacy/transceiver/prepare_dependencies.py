#!/usr/bin/env python3
"""Fetch the original PHY dependencies, or verify manually supplied checkouts."""
import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Verify without downloading')
    args = parser.parse_args()
    dependencies = json.loads((ROOT / 'dependencies.json').read_text())['git_dependencies']
    for dep in dependencies:
        path = ROOT / dep['path']
        if not path.exists():
            if args.check:
                parser.error(f"Missing {path}; supply {dep['url']} at {dep['commit']}")
            subprocess.run(['git', 'clone', '--no-checkout', dep['url'], str(path)], check=True)
            subprocess.run(['git', '-C', str(path), 'checkout', '--detach', dep['commit']], check=True)
            subprocess.run(['git', '-C', str(path), 'submodule', 'update', '--init', '--recursive'], check=True)
        # Never overwrite a developer's existing checkout or local modifications.
        if not (path / '.git').exists():
            parser.error(f'{path} must be a dependency Git checkout')
        revision = subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()
        if revision != dep['commit']:
            parser.error(f"{path}: expected {dep['commit']}, found {revision}")
        print(f"{dep['path']}: {revision}")


if __name__ == '__main__':
    main()
