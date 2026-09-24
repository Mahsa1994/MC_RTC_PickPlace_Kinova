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
# Text unique to the joint-limit fix; presence in kinova.cpp means this
# source tree already has it (see patches/mc_kinova/mc_kinova_6dof.patch).
MARKER="infinite-rotation actuators"
# Text unique to the pre-existing 6DOF scaffolding (dof6 param etc.), which
# on this image's containers already exists *before* this script ever runs -
# it predates the joint-limit fix and was not added by this script/patch.
SCAFFOLD_MARKER="bool dof6"

log()  { echo "[apply_mc_kinova_patch] $*"; }
err()  { echo "[apply_mc_kinova_patch] ERROR: $*" >&2; }

if [[ ! -f "$PATCH_FILE" ]]; then
  err "patch file not found: $PATCH_FILE"
  exit 1
fi

# -----------------------------------------------------------------------------
# 1. Patch the canonical source tree (devel/mc_kinova). Idempotent via MARKER.
#
#    Three possible starting states, handled separately because a plain
#    `git apply` of the bundled patch only works cleanly against a vanilla
#    upstream checkout (state C below) - most containers built from this
#    image already have state B, where git apply's context doesn't match
#    and 3-way merging isn't reliable either:
#      A. already has the joint-limit fix (MARKER present)      -> skip
#      B. has the pre-existing 6DOF scaffolding but not the fix
#         (SCAFFOLD_MARKER present, MARKER absent)              -> targeted
#                                                                   text
#                                                                   surgery
#      C. vanilla upstream mc_kinova, no 6DOF scaffolding at all -> git apply
#         the full bundled patch (adds scaffolding + the fix together)
# -----------------------------------------------------------------------------
DEVEL_SRC="$WORKSPACE_DIR/devel/mc_kinova"
KINOVA_CPP="$DEVEL_SRC/src/kinova.cpp"

if [[ ! -d "$DEVEL_SRC" ]]; then
  err "$DEVEL_SRC not found - is the mc_rtc superbuild set up (devel/mc_kinova checked out)?"
  exit 1
fi

git_devel() { git -c safe.directory='*' -C "$DEVEL_SRC" "$@"; }

# The exact joint-limit block as it exists before the fix (shared verbatim by
# the 7DOF and 6DOF variants until this fix teaches it to tell them apart).
# Must match patches/mc_kinova/mc_kinova_6dof.patch's corresponding hunk.
ORIG_BLOCK='  else
  {
    update_joint_limit("joint_2", -2.15, 2.15);
    update_joint_limit("joint_4", -2.45, 2.45);
    update_joint_limit("joint_6", -2.0, 2.0);
  }'
FIXED_BLOCK='  else if(dof6)
  {
    // Gen3 6DOF: joint_1, joint_4 and joint_6 are infinite-rotation actuators
    // (continuous in kinova_6dof.urdf), so keep their unbounded URDF limits.
    // joint_3 (+-2.57) and joint_5 (+-2.09) already come from the URDF.
    update_joint_limit("joint_2", -2.15, 2.15);
  }
  else
  {
    update_joint_limit("joint_2", -2.15, 2.15);
    update_joint_limit("joint_4", -2.45, 2.45);
    update_joint_limit("joint_6", -2.0, 2.0);
  }'

if grep -q "$MARKER" "$KINOVA_CPP" 2>/dev/null; then
  log "devel/mc_kinova already patched (state A), skipping."

elif grep -q "$SCAFFOLD_MARKER" "$DEVEL_SRC/src/kinova.h" 2>/dev/null; then
  log "devel/mc_kinova has 6DOF scaffolding but not the joint-limit fix (state B) - applying targeted fix ..."
  python3 - "$KINOVA_CPP" <<PYEOF
import sys
path = sys.argv[1]
orig = """$ORIG_BLOCK"""
fixed = """$FIXED_BLOCK"""
text = open(path).read()
n = text.count(orig)
if n != 1:
    print(f"expected exactly 1 occurrence of the original joint-limit block, found {n}", file=sys.stderr)
    sys.exit(1)
open(path, "w").write(text.replace(orig, fixed, 1))
PYEOF
  if [[ $? -ne 0 ]]; then
    err "targeted text substitution failed - $KINOVA_CPP doesn't match the expected pre-fix text."
    err "Diff it by hand against patches/mc_kinova/mc_kinova_6dof.patch, see $PATCH_DIR/README.md."
    exit 1
  fi
  if ! grep -q "$MARKER" "$KINOVA_CPP"; then
    err "substitution ran but marker text still missing - inspect $KINOVA_CPP"
    exit 1
  fi
  log "devel/mc_kinova patched (targeted)."

else
  log "devel/mc_kinova looks like a vanilla upstream checkout (state C) - applying full patch ..."
  patch_err="$(git_devel apply --check "$PATCH_FILE" 2>&1)"
  if [[ -z "$patch_err" ]]; then
    git_devel apply "$PATCH_FILE"
  else
    log "plain git apply --check failed, trying -3way. (reason: $patch_err)"
    patch_err3="$(git_devel apply --check -3 "$PATCH_FILE" 2>&1)"
    if [[ -z "$patch_err3" ]]; then
      git_devel apply -3 "$PATCH_FILE"
    else
      err "patch does not apply to $DEVEL_SRC (expected upstream baseline: $UPSTREAM_PIN)."
      err "git apply error: $patch_err"
      err "git apply -3 error: $patch_err3"
      err "Apply it by hand, see $PATCH_DIR/README.md."
      exit 1
    fi
  fi
  if ! grep -q "$MARKER" "$KINOVA_CPP"; then
    err "patch applied but marker text still missing - inspect $KINOVA_CPP"
    exit 1
  fi
  log "devel/mc_kinova patched (full patch)."
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
