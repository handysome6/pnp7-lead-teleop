"""Save a freshly measured joint pose without regenerating a teleop config."""
from __future__ import annotations

import math
import os
import re
import stat
import tempfile
from pathlib import Path
from typing import Callable

from gui.session import Rejected


def save_current_home(config: Path, read_q: Callable[[], list[float]]) -> dict:
    original = config.read_bytes()
    original_stat = config.stat()
    q = read_q()
    if (not isinstance(q, list) or len(q) != 7
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in q)):
        raise Rejected("机器人没有返回 7 个有效关节角，配置未修改")
    q = [float(v) for v in q]
    values = " ".join(format(v, ".17g") for v in q)
    text = original.decode("utf-8")
    lines = text.splitlines(keepends=True)
    found = False
    for i, line in enumerate(lines):
        body = line.rstrip("\r\n")
        ending = line[len(body):]
        match = re.fullmatch(r"([ \t]*home_qpos[ \t]*=[ \t]*)([^#]*)(#.*)?", body)
        if match:
            # Update every active occurrence: the C++ parser uses the last one.
            comment = match[3] or ""
            spacing = re.search(r"[ \t]*$", match[2])[0] if comment else ""
            lines[i] = match[1] + values + spacing + comment + ending
            found = True
    if not found:
        newline = "\r\n" if "\r\n" in text else "\n"
        if lines and not lines[-1].endswith(("\r", "\n")):
            lines[-1] += newline
        lines.append("home_qpos=" + values + newline)
    updated = "".join(lines).encode("utf-8")

    # A slow/unavailable FCI must not overwrite an edit made while reading it.
    current_stat = config.stat()
    identity = lambda s: (s.st_dev, s.st_ino, s.st_mtime_ns, s.st_ctime_ns, s.st_size)
    if (config.is_symlink() or identity(current_stat) != identity(original_stat)
            or config.read_bytes() != original):
        raise Rejected("读取姿态期间配置已被修改，请重新保存 Home")
    mode = stat.S_IMODE(original_stat.st_mode)
    backup = config.with_name(config.name + ".home.bak")
    _atomic_write(backup, original, mode)
    _atomic_write(config, updated, mode)
    return {"config_name": config.stem, "path": str(config),
            "backup": str(backup), "q": q}


def _atomic_write(path: Path, data: bytes, mode: int) -> None:
    temp = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".home-", delete=False) as fh:
            temp = Path(fh.name)
            os.fchmod(fh.fileno(), mode)
            fh.write(data)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(temp, path)
    finally:
        if temp is not None:
            temp.unlink(missing_ok=True)
