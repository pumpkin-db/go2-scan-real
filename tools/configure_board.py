#!/usr/bin/env python3
"""Create the machine-local OK3588 connection file without exposing secrets."""
import argparse
import ast
import getpass
import json
import os
from pathlib import Path
import tempfile


ROOT = Path(__file__).resolve().parents[1]
DESTINATION = ROOT / 'config/local/board.json'


def import_oksh(path):
    values = {}
    tree = ast.parse(path.read_text())
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id in ('host', 'user', 'pw'):
                values[target.id] = ast.literal_eval(node.value)
    if not {'host', 'user'} <= values.keys():
        raise RuntimeError(f'cannot read host/user from {path}')
    return {
        'host': values['host'], 'port': 22, 'user': values['user'],
        'password': values.get('pw', '')
    }


def interactive():
    host = input('OK3588 host [192.168.123.30]: ').strip() or '192.168.123.30'
    user = input('SSH user [root]: ').strip() or 'root'
    password = getpass.getpass('SSH password (blank = SSH key): ')
    return {'host': host, 'port': 22, 'user': user, 'password': password}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--import-oksh', type=Path)
    args = parser.parse_args()
    config = import_oksh(args.import_oksh) if args.import_oksh else interactive()
    DESTINATION.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.board.', dir=str(DESTINATION.parent))
    try:
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, 'w') as stream:
            json.dump(config, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
        os.replace(temporary, DESTINATION)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    print(f'Wrote {DESTINATION} with mode 0600; this file is gitignored.')


if __name__ == '__main__':
    main()
