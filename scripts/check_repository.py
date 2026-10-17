#!/usr/bin/env python3
"""Check first-party tracked files and local Markdown links before publishing."""
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]
FORBIDDEN = {'.agents', '.codex', '.opencode', '.sisyphus', '.vscode', '__pycache__',
             'AGENTS.md', 'CLAUDE.md', 'SKILL.md', '笔记.md', '.env', 'superpowers',
             'brainstorms', 'plans'}
BINARY_SUFFIXES = {'.o', '.a', '.so', '.pyc', '.zip', '.tar', '.gz', '.log', '.pem', '.key'}
TOKEN = re.compile(r'gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{50,}|-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----')


def main():
    entries = subprocess.check_output(['git', 'ls-files', '--stage', '-z'], cwd=ROOT).decode().split('\0')
    errors = []
    count = 0
    for entry in filter(None, entries):
        metadata, name = entry.split('\t', 1)
        mode = metadata.split()[0]
        if mode == '160000':
            continue  # Dependency contents belong to their pinned upstream repository.
        path = ROOT / name
        parts = Path(name).parts
        if any(part in FORBIDDEN or part.startswith('arm_control_sdk') or part.startswith('build-') for part in parts):
            errors.append(name + ': local-only content')
        if path.suffix in BINARY_SUFFIXES:
            errors.append(name + ': binary, secret or generated artifact')
        if not path.exists():
            errors.append(name + ': file missing')
            continue
        if path.is_symlink():
            continue
        count += 1
        # Screenshots are intentional documentation assets, not build artifacts.
        if parts[:2] == ('docs', 'assets') and path.suffix == '.png':
            with path.open('rb') as asset:
                if asset.read(8) != b'\x89PNG\r\n\x1a\n':
                    errors.append(name + ': invalid PNG signature')
            continue
        try:
            text = path.read_text()
        except UnicodeDecodeError:
            errors.append(name + ': unexpected binary')
            continue
        if TOKEN.search(text):
            errors.append(name + ': possible credential')
        if path.suffix == '.md':
            for link in re.findall(r'\]\(([^)]+)\)', text):
                link = link.strip('<>')
                url = urlsplit(link)
                if url.scheme or not url.path:
                    continue
                target = path.parent / unquote(url.path)
                if not target.exists():
                    errors.append(name + ': broken local link: ' + link)
        if path.suffix in {'.md', '.yaml', '.yml'} and '/Users/' in text:
            errors.append(name + ': machine-specific home path')
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        return 1
    print('Repository checks passed: {} source/doc/asset files; no forbidden tracked artifacts; local links valid.'.format(count))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
