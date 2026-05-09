#!/usr/bin/env bash
# Tolerance-based regression check.
#
# Quatro is a randomized algorithm: KdTreeFLANN's randomized kd-tree splits and
# the PMC heuristic max-clique solver produce slightly different outputs across
# runs, even with OMP_NUM_THREADS=1 (the README acknowledges "multi-thread
# issues" on the first run, but the variance is wider than just that).
# An exact-string `diff` against a frozen baseline therefore can't pass even at
# HEAD with no code changes. Instead this script runs the example once and
# checks that the resulting 4x4 matrix is consistent with the algorithm's
# observed pre-refactor noise band on the bundled toy KITTI pair (000540 ->
# 001319, expected yaw ~28°, translation ~-8.7m in x).
#
# Bounds were derived from 12+ pre-refactor runs (OMP=1 and OMP=4 mixed) and
# deliberately widened to absorb the algorithm's natural variance with margin:
#   m00 (cos yaw):          0.85 .. 0.92    (solution always near yaw = 28 deg)
#   m03 (tx):              -10.0 .. -7.0    (translation always large negative)
#   m13 (ty):               -3.0 .. +3.0    (FLANN kd-tree variance affects this)
#   m20, m21, m22:          0, 0, 1         (Quatro is yaw-only by construction)
#   m23 (tz):               -3.0 .. +3.0
#   m30, m31, m32, m33:     0, 0, 0, 1
#
# Bounds are loose enough not to flake on legitimate runs and tight enough to
# catch a refactor that broke the core pipeline (identity output, wrong sign,
# 90-degree-off solution, NaN, etc). They are NOT a precision check.
#
# Exits 0 on pass, non-zero on fail.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

LINE="$("$SCRIPT_DIR/run.sh" test 2>&1 | grep -m1 '^\[QUATRO_OUTPUT\]' || true)"

if [[ -z "$LINE" ]]; then
    echo "FAIL: no [QUATRO_OUTPUT] line emitted by the example." >&2
    exit 2
fi

echo "Got: $LINE"

read -r tag m00 m01 m02 m03 m10 m11 m12 m13 m20 m21 m22 m23 m30 m31 m32 m33 <<< "$LINE"

check_range() {
    local name="$1" value="$2" lo="$3" hi="$4"
    awk -v n="$name" -v v="$value" -v lo="$lo" -v hi="$hi" '
        BEGIN {
            if (v+0 < lo+0 || v+0 > hi+0) {
                printf("FAIL: %s = %s, expected %s .. %s\n", n, v, lo, hi) > "/dev/stderr"
                exit 1
            } else {
                printf("ok:   %s = %s in [%s, %s]\n", n, v, lo, hi)
            }
        }
    '
}

check_eq() {
    local name="$1" value="$2" expected="$3"
    awk -v n="$name" -v v="$value" -v e="$expected" '
        BEGIN {
            d = v + 0 - e
            if (d < 0) d = -d
            if (d > 1e-6) {
                printf("FAIL: %s = %s, expected exactly %s\n", n, v, e) > "/dev/stderr"
                exit 1
            } else {
                printf("ok:   %s = %s == %s\n", n, v, e)
            }
        }
    '
}

failures=0
check_range "m00 (cos yaw)" "$m00"  0.85  0.92 || failures=$((failures+1))
check_range "m03 (tx)"      "$m03" -10.0 -7.0  || failures=$((failures+1))
check_range "m13 (ty)"      "$m13"  -3.0  3.0  || failures=$((failures+1))
check_range "m23 (tz)"      "$m23"  -3.0  3.0  || failures=$((failures+1))
check_eq    "m20"           "$m20"  0  || failures=$((failures+1))
check_eq    "m21"           "$m21"  0  || failures=$((failures+1))
check_eq    "m22"           "$m22"  1  || failures=$((failures+1))
check_eq    "m30"           "$m30"  0  || failures=$((failures+1))
check_eq    "m31"           "$m31"  0  || failures=$((failures+1))
check_eq    "m32"           "$m32"  0  || failures=$((failures+1))
check_eq    "m33"           "$m33"  1  || failures=$((failures+1))

# Sanity: rotation block determinant should be +1 (no reflection).
det_check=$(awk -v a="$m00" -v b="$m01" -v c="$m10" -v d="$m11" '
    BEGIN {
        det = a*d - b*c
        diff = det - 1
        if (diff < 0) diff = -diff
        if (diff > 0.01) {
            printf("FAIL: 2x2 R block det = %.4f, expected ~1.0\n", det) > "/dev/stderr"
            exit 1
        }
        printf("ok:   2x2 R block det = %.4f (~ 1.0)\n", det)
    }
') || failures=$((failures+1))
echo "$det_check"

if [[ $failures -ne 0 ]]; then
    echo "REGRESSION FAILED ($failures check(s) violated)"
    exit 1
fi

echo "REGRESSION OK"
