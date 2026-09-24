#!/usr/bin/env bash
# =============================================================================
#  apply_mc_kinova_patch.sh
#
#  Applies patches/mc_kinova/mc_kinova_6dof.patch (Kinova6DOF module +
#  unbounded joint_4/joint_6 limits, see patches/mc_kinova/README.md) to the
#  mc_kinova source tree(s) in this container, rebuilds, and installs.
#
#  Run this INSIDE the docker container, any time after a fresh checkout of
#  this repo (or after editing the patch itself), before starting the
#  admittance bridge. Safe to re-run: every step checks whether it's already
#  done and skips it.
#
#  Usage: src/apply_mc_kinova_patch.sh
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"          # .../workspace, parent of src/
PATCH_DIR="$WORKSPACE_DIR/patches/mc_kinova"
PATCH_FILE="$PATCH_DIR/mc_kinova_6dof.patch"
SHARE_DIR="$PATCH_DIR/share"
UPSTREAM_PIN="7cf7424"
# Text unique to the joint-limit fix hunk; presence in kinova.cpp means this
# source tree already has the patch (see patches/mc_kinova/mc_kinova_6dof.patch).
MARKER="infinite-rotation actuators"

log()  { echo "[apply_mc_kinova_patch] $*"; }
err()  { echo "[apply_mc_kinova_patch] ERROR: $*" >&2; }

if [[ ! -f "$PATCH_FILE" ]]; then
  err "patch file not found: $PATCH_FILE"
  exit 1
fi

# -----------------------------------------------------------------------------
# 1. Patch the canonical source tree (devel/mc_kinova). Idempotent via MARKER.
# -----------------------------------------------------------------------------
DEVEL_SRC="$WORKSPACE_DIR/devel/mc_kinova"

if [[ ! -d "$DEVEL_SRC" ]]; then
  err "$DEVEL_SRC not found - is the mc_rtc superbuild set up (devel/mc_kinova checked out)?"
  exit 1
fi

git_devel() { git -c safe.directory='*' -C "$DEVEL_SRC" "$@"; }

if grep -q "$MARKER" "$DEVEL_SRC/src/kinova.cpp" 2>/dev/null; then
  log "devel/mc_kinova already patched, skipping git apply"
else
  log "patching devel/mc_kinova ..."
  if git_devel apply --check "$PATCH_FILE" 2>/dev/null; then
    git_devel apply "$PATCH_FILE"
  elif git_devel apply --check -3 "$PATCH_FILE" 2>/dev/null; then
    # -3way: tolerates a tree that's already partway to the patched state
    # (e.g. this image's baseline 6DOF scaffolding predates the joint-limit
    # fix) by merging on the blob level instead of failing outright.
    git_devel apply -3 "$PATCH_FILE"
  else
    err "patch does not apply to $DEVEL_SRC (expected upstream baseline: $UPSTREAM_PIN)."
    err "Apply it by hand, see $PATCH_DIR/README.md."
    exit 1
  fi
  if ! grep -q "$MARKER" "$DEVEL_SRC/src/kinova.cpp"; then
    err "patch applied but marker text still missing - inspect $DEVEL_SRC/src/kinova.cpp"
    exit 1
  fi
  log "devel/mc_kinova patched."
fi

# -----------------------------------------------------------------------------
# 2. Sync the patched sources into every other mc_kinova checkout used as a
#    separate CMake source tree (not a symlink/same-dir build), then rebuild
#    + install each one that exists in this container.
# -----------------------------------------------------------------------------
SUPERBUILD_SRC="$WORKSPACE_DIR/build/superbuild/src/mc_kinova"     # -> /usr/local
SUPERBUILD_BUILD="$WORKSPACE_DIR/build/superbuild/build/mc_kinova"
PROJECTS_BUILD="$WORKSPACE_DIR/build/projects/mc_kinova"           # source IS devel/mc_kinova -> $WORKSPACE_DIR/install

rebuild_and_install() {
  local build_dir=$1 label=$2
  if [[ ! -d "$build_dir" ]]; then
    log "$label build dir not found ($build_dir), skipping."
    return 0
  fi
  log "building $label ($build_dir) ..."
  cmake --build "$build_dir" -j"$(nproc)"
  log "installing $label ..."
  cmake --install "$build_dir"
}

if [[ -d "$SUPERBUILD_SRC" ]]; then
  if grep -q "$MARKER" "$SUPERBUILD_SRC/src/kinova.cpp" 2>/dev/null; then
    log "build/superbuild/src/mc_kinova already has the patched sources."
  else
    log "syncing patched sources into build/superbuild/src/mc_kinova ..."
    cp "$DEVEL_SRC/src/kinova.cpp"  "$SUPERBUILD_SRC/src/kinova.cpp"
    cp "$DEVEL_SRC/src/kinova.h"    "$SUPERBUILD_SRC/src/kinova.h"
    cp "$DEVEL_SRC/src/module.cpp"  "$SUPERBUILD_SRC/src/module.cpp"
  fi
  rebuild_and_install "$SUPERBUILD_BUILD" "mc_kinova (/usr/local)"
else
  log "build/superbuild/src/mc_kinova not found, skipping (/usr/local install)."
fi

rebuild_and_install "$PROJECTS_BUILD" "mc_kinova (\$WORKSPACE/install)"

# -----------------------------------------------------------------------------
# 3. Make sure the 6DOF URDF/RSDF/convex-hull assets are present in every
#    installed share/ tree mc_rtc might read from. CMake does not generate
#    these (see patches/mc_kinova/README.md), so they're copied verbatim.
# -----------------------------------------------------------------------------
install_share() {
  local prefix=$1
  if [[ ! -d "$prefix" ]]; then
    return 0
  fi
  log "syncing 6DOF share assets into $prefix/share/mc_kinova ..."
  mkdir -p "$prefix/share/mc_kinova"
  cp -r "$SHARE_DIR"/. "$prefix/share/mc_kinova/"
}

install_share "/usr/local"
install_share "$WORKSPACE_DIR/install"

log "done."
