#!/usr/bin/env bash
# Build the space-time planner's Rust extension INSIDE the container.
#
# The planner (mounted at /ws/planner_src, see PLANNER_REPO in the Makefile)
# has no Python solver: optimize_spacetime() raises unless `bezier_opt`, a
# pyo3 extension around clarabel, imports. This builds it against THIS
# container's interpreter and installs it under /ws/install/planner_py, a
# persisted volume, which /etc/profile.d puts on PYTHONPATH for every shell.
#
# The build is the claim; the import at the end is the check. maturin will
# happily build against another interpreter and report success -- the
# planner's own README records being burned by exactly that -- so the last
# line imports the module from the path it was installed to and prints where
# it came from.
set -o pipefail
source /opt/ros/jazzy/setup.bash
cd /ws

SRC=/ws/planner_src
if [ ! -f "$SRC/rust_optimizer/pybind/Cargo.toml" ]; then
  echo "FAIL: no planner at $SRC (rust_optimizer/pybind/Cargo.toml missing)."
  echo "      Set PLANNER_REPO on the host and recreate the container: make planner-sync && make up"
  exit 1
fi

OUT=/ws/build/bezier_opt
DEST=/ws/install/planner_py
mkdir -p "$OUT" "$DEST"

echo "== building bezier_opt (release) against $(python3 --version) =="
# PYO3_USE_ABI3_FORWARD_COMPATIBILITY: pyo3 0.24 refuses interpreters newer
# than it knows unless told to build against the stable ABI (planner README).
# CARGO_TARGET_DIR keeps the 1 GB of build products in the container's own
# volume rather than in the bind-mounted source tree.
( cd "$SRC/rust_optimizer/pybind" \
  && PYO3_PYTHON="$(command -v python3)" \
     PYO3_USE_ABI3_FORWARD_COMPATIBILITY=1 \
     CARGO_TARGET_DIR=/ws/build/cargo_target \
     maturin build --release --interpreter "$(command -v python3)" --out "$OUT" ) \
  || { echo "FAIL: maturin build failed"; exit 1; }

WHEEL="$(ls -t "$OUT"/bezier_opt-*.whl | head -1)"
echo "== installing $WHEEL -> $DEST =="
pip install --quiet --break-system-packages --no-deps --force-reinstall --target "$DEST" "$WHEEL" \
  || { echo "FAIL: pip install failed"; exit 1; }

echo "== checking the import (from the installed path, not the source tree) =="
PYTHONPATH="$DEST:$SRC" python3 - <<'PY' || exit 1
import bezier_opt, spacetime_bezier, sys
path = bezier_opt.__file__
assert path.startswith("/ws/install/planner_py"), f"bezier_opt imported from {path}, not the install"
assert hasattr(bezier_opt, "optimize_spacetime_bezier"), "extension has no optimize_spacetime_bezier"
print(f"  bezier_opt        {path}")
print(f"  spacetime_bezier  {spacetime_bezier.__file__}")
print(f"  python            {sys.version.split()[0]}")
PY
echo "PASS: the planner's solver is built and imports. Try: make solve SCENARIO=fence3d N=8 NSEG=2"
