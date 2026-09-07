#!/usr/bin/env python3
"""R1.1 audit: advertised entry points vs the installed index.

Checks, for every package under src/:
  1. console_scripts declared in setup.py/setup.cfg -> install/<pkg>/lib/<pkg>/<name>
  2. ament_cmake install(TARGETS ... DESTINATION lib/${PROJECT_NAME})
       - runtime targets must be in install/<pkg>/lib/<pkg>/
       - plugin libraries (*.so) must be in install/<pkg>/lib/
Prints one line per mismatch. Exit code 0 = clean, 1 = mismatches found.
"""
import configparser
import re
import sys
from pathlib import Path

WS = Path(__file__).resolve().parent.parent
SRC = WS / "src"
INSTALL = WS / "install"

# runtime target names: tokens before ARCHIVE/LIBRARY/RUNTIME keywords or a
# bare DESTINATION; e.g.  install(TARGETS a b ARCHIVE DESTINATION lib ...)
TARGET_BLOCK_RE = re.compile(
    r"install\s*\(\s*TARGETS\s+(.*?)\)",
    re.MULTILINE | re.DOTALL)
KEYWORDS = {"ARCHIVE", "LIBRARY", "RUNTIME", "DESTINATION", "EXPORT",
            "INCLUDES", "OPTIONAL", "FRAMEWORK", "BUNDLE", "PUBLIC_HEADER",
            "RESOURCE", "PRIVATE_HEADER", "OBJECT"}


def runtime_targets(block):
    """Return target names from an install(TARGETS ...) argument block."""
    tokens = block.replace("\n", " ").split()
    names = []
    for tok in tokens:
        if tok.upper() in KEYWORDS:
            break
        names.append(tok)
    return names


def strip_cmake_comments(text):
    """Remove full-line and trailing '#' comments (naive but adequate)."""
    return "\n".join(line.split("#", 1)[0] for line in text.splitlines())


def block_has_runtime_keyword(block):
    return re.search(r"\bRUNTIME\b", block) is not None


def setup_entry_points(pkg_dir):
    """Return dict entry_name -> module:func from setup.py + setup.cfg."""
    eps = {}
    setup_py = pkg_dir / "setup.py"
    if setup_py.is_file():
        text = setup_py.read_text(errors="replace")
        m = re.search(r"entry_points\s*=\s*\{(.*?)\n\s*\}", text, re.DOTALL)
        if m:
            for name, target in re.findall(
                    r"['\"]([\w\-]+)['\"]\s*[:=]\s*['\"]([\w\.:\-]+)['\"]", m.group(1)):
                eps[name] = target
    cfg = pkg_dir / "setup.cfg"
    if cfg.is_file():
        cp = configparser.ConfigParser()
        cp.read(cfg)
        if cp.has_option("entry_points", "console_scripts"):
            for line in cp.get("entry_points", "console_scripts").splitlines():
                line = line.strip().rstrip(",")
                if "=" in line:
                    name, target = [p.strip() for p in line.split("=", 1)]
                    eps.setdefault(name, target)
    return eps


def data_files_installed(pkg_dir):
    problems = []
    setup_py = pkg_dir / "setup.py"
    if setup_py.is_file():
        text = setup_py.read_text(errors="replace")
        m = re.search(r"data_files\s*=\s*\[(.*?)\n\s*\]", text, re.DOTALL)
        if m:
            for dest, src_glob in re.findall(
                    r"\(\s*['\"]([^'\"]+)['\"]\s*,\s*\[([^\]]+)\]", m.group(1)):
                for tok in re.findall(r"['\"]([^'\"]+)['\"]", src_glob):
                    base = tok.split("/", 1)[0]
                    if not (pkg_dir / base).exists():
                        problems.append(f"data_file source missing: {tok} (dest {dest})")
    return problems


def main():
    report = []
    packages = sorted({px.parent for px in SRC.rglob("package.xml")
                       if "test" not in px.parts})
    for pkg_dir in packages:
        pkg = pkg_dir.name
        lib_pkg = INSTALL / pkg / "lib" / pkg

        for name in sorted(setup_entry_points(pkg_dir)):
            if not (lib_pkg / name).exists():
                report.append(f"[MISSING-EXEC] {pkg}: '{name}' not at "
                              f"{lib_pkg.relative_to(WS)}/{name}")

        cmake = pkg_dir / "CMakeLists.txt"
        if cmake.is_file():
            text = strip_cmake_comments(cmake.read_text(errors="replace"))
            for m in TARGET_BLOCK_RE.finditer(text):
                block = m.group(1)
                for tgt in runtime_targets(block):
                    if "${" in tgt or tgt in ("#",):
                        continue  # variable/placeholder target names
                    exec_ok = (lib_pkg / tgt).exists()
                    plugin_ok = (INSTALL / pkg / "lib" / f"lib{tgt}.so").exists()
                    if exec_ok or plugin_ok:
                        continue
                    if not (INSTALL / pkg).exists():
                        report.append(
                            f"[NOT-BUILT] {pkg}: package not in install space; "
                            f"target '{tgt}' unresolved (optional package?)")
                    else:
                        report.append(
                            f"[MISSING-TARGET] {pkg}: '{tgt}' neither at "
                            f"{lib_pkg.relative_to(WS)}/{tgt} nor as plugin "
                            f"lib{tgt}.so")

        for problem in data_files_installed(pkg_dir):
            report.append(f"[MISSING-DATAFILE] {pkg}: {problem}")

    print(f"Packages audited: {len(packages)}")
    if report:
        print(f"Mismatches: {len(report)}")
        for line in report:
            print(" ", line)
        Path("/tmp/r1_1_audit.txt").write_text("\n".join(report) + "\n")
        return 1
    print("All advertised entry points resolve to the installed index.")
    Path("/tmp/r1_1_audit.txt").write_text("")
    return 0


if __name__ == "__main__":
    sys.exit(main())
