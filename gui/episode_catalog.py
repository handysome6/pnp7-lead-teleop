"""Disk-backed saved-take counts and monotonically reserved episode IDs."""
from __future__ import annotations

import fcntl
import json
import os
import re
import tempfile
from pathlib import Path


def _entries(root: Path, prefix: str):
    if not re.fullmatch(r'[A-Za-z0-9_-]+', prefix):
        raise ValueError('episode prefix 只能包含字母、数字、下划线或连字符')
    pattern = re.compile(re.escape(prefix) + r'(\d+)')
    return [(int(m[1]), p) for p in root.iterdir()
            if (m := pattern.fullmatch(p.name))] if root.exists() else []


def _sequence(root: Path) -> dict:
    path = root / '.pnp7_episode_sequence.json'
    if not path.exists():
        return {}
    value = json.loads(path.read_text())
    if (not isinstance(value, dict) or any(not isinstance(k, str) or
            type(v) is not int or v < 0 for k, v in value.items())):
        raise ValueError(f'采集编号记录损坏：{path}')
    return value


def progress(root: Path, prefix: str) -> dict:
    entries = _entries(root, prefix)
    last = max([_sequence(root).get(prefix, 0)] + [i for i, _ in entries])
    kept = 0
    incomplete = 0
    for _, ep in entries:
        if not ep.is_dir() or ep.is_symlink():
            continue
        try:
            meta = json.loads((ep / 'episode_meta.json').read_text())
            complete = (not (ep / 'failure.json').exists() and
                        isinstance(meta, dict) and type(meta.get('frames')) is int and
                        meta['frames'] > 0 and (ep / 'episode.csv').is_file())
            if complete:
                with (ep / 'episode.csv').open() as fh:
                    complete = bool(fh.readline().strip() and fh.readline().strip())
        except (OSError, ValueError):
            complete = False
        kept += int(complete)
        incomplete += int(not complete)
    return {'episode_index': kept, 'next_episode': f'{prefix}{last + 1:03d}',
            'incomplete_episodes': incomplete, 'dataset_dir': str(root)}


def reserve(root: Path, prefix: str) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    with (root / '.pnp7_episode_sequence.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        name = progress(root, prefix)['next_episode']
        index = int(name[len(prefix):])
        sequence = _sequence(root)
        sequence[prefix] = index
        temp = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=root, prefix='.sequence-', delete=False) as fh:
                temp = Path(fh.name)
                json.dump(sequence, fh)
                fh.write('\n')
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(temp, root / '.pnp7_episode_sequence.json')
        finally:
            if temp is not None:
                temp.unlink(missing_ok=True)
        episode = root / name
        episode.mkdir()  # Never merge with/overwrite an existing directory.
        return episode
