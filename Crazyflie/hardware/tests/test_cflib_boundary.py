"""Boundary test: only cflib_connection.py may import cflib."""

import ast
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_SCANNED_DIRS = ("Crazyflie", "scripts")
_ALLOWED = _REPO_ROOT / "Crazyflie" / "hardware" / "cflib_connection.py"


def _is_test_file(path: Path) -> bool:
    return path.name == "conftest.py" or path.name.startswith("test_") or "tests" in path.parts


def _cflib_imports(path: Path) -> list[str]:
    """Return a 'file:line: statement' entry for every cflib import in path."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    found: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom) and node.module is not None and node.level == 0:
            names = [node.module]
        else:
            continue
        found.extend(
            f"{path.relative_to(_REPO_ROOT)}:{node.lineno}: {name}"
            for name in names
            if name == "cflib" or name.startswith("cflib.")
        )
    return found


def test_only_cflib_connection_imports_cflib():
    offenders: list[str] = []
    for directory in _SCANNED_DIRS:
        for path in sorted((_REPO_ROOT / directory).rglob("*.py")):
            if path == _ALLOWED or _is_test_file(path) or "logs" in path.parts:
                continue
            offenders.extend(_cflib_imports(path))

    assert offenders == [], "cflib imported outside cflib_connection.py:\n" + "\n".join(offenders)
