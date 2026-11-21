#!/bin/bash
# Plain-text copy of every published rawlog, compressed, into $OUT/text/<world>/<traj>/<variant>/.
# Resumable: a folder with a .done marker is skipped. One job per rawlog, $JOBS in parallel,
# pinned to $CORES at low priority (the converter and its compressor take about one core each).
#
#   export_text.sh [world/traj ...]     (default: every sequence in MANIFEST.tsv)
# ROS setup scripts read unset variables: source them before "set -u".
source "$HOME/mvsim_lio_benchmark/tools/env.sh" >/dev/null 2>&1
set -u
DATA=${DATA:-/mnt/datasets/mvsim_lio_benchmark}
OUT=${OUT:-$HOME/hf_stage/text_copy}
CORES=${CORES:-0-4,23-25,44-48,67-69}
JOBS=${JOBS:-14}
COMPRESS=${COMPRESS:-xz -9 -T1}
EXT=${EXT:-.xz}
TOOL=${TOOL:-$(dirname "$(readlink -f "$0")")/../build/rawlog_tools/rawlog-to-text}
MANIFEST=${MANIFEST:-$HOME/mvsim_lio_benchmark/MANIFEST.tsv}
LOG=${LOG:-$HOME/txt_export/export.log}

one() {
  local rel=$1                       # <world>/<traj>/<variant>.rawlog
  local seq=${rel%/*}
  local var=${rel##*/}
  var=${var%.rawlog}
  local dst=$OUT/text/$seq/$var
  [ -e "$dst/.done" ] && return 0
  rm -rf "$dst" && mkdir -p "$dst"
  local t0=$SECONDS
  if ! out=$("$TOOL" "$DATA/$rel" "$dst" "$COMPRESS" "$EXT" 2>&1); then
    echo "$(date -u +%FT%TZ) FAILED $seq/$var: $out" >> "$LOG"
    return 1
  fi
  xz -9 -T1 "$dst/sweeps.txt" "$dst/imu.txt" "$dst/odometry.txt" || return 1
  for f in gt.tum lidar1.gt.tum errors.json counts.json; do
    [ -e "$DATA/$seq/$var.$f" ] && cp "$DATA/$seq/$var.$f" "$dst/$f"
  done
  # The counts must match the rawlog's own counts.json.
  local want
  want=$(python3 -c "import json,sys; c=json.load(open(sys.argv[1])); print(c['sweeps'], c['points'])" "$dst/counts.json")
  local got
  got=$(echo "$out" | awk '{print $2, $4}')
  if [ "$want" != "$got" ]; then
    echo "$(date -u +%FT%TZ) MISMATCH $seq/$var: counts.json '$want', exported '$got'" >> "$LOG"
    return 1
  fi
  touch "$dst/.done"
  echo "$(date -u +%FT%TZ) done $seq/$var ($((SECONDS - t0)) s): $out" >> "$LOG"
}
export -f one
export DATA OUT COMPRESS EXT TOOL LOG

mkdir -p "$OUT/text"
if [ $# -gt 0 ]; then
  pat=$(printf '%s/|' "$@")
  pat="^(${pat%|})"
else
  pat='.'
fi
# Per sequence, the shared 200 Hz ground truth.
cut -f1 "$MANIFEST" | grep -E "$pat" | grep '/gt_200hz.tum$' | while read -r rel; do
  mkdir -p "$OUT/text/${rel%/*}"
  [ -e "$OUT/text/$rel.xz" ] || xz -9 -T1 -c "$DATA/$rel" > "$OUT/text/$rel.xz"
done
echo "$(date -u +%FT%TZ) start: $(cut -f1 "$MANIFEST" | grep -E "$pat" | grep -c '\.rawlog$') rawlogs, $JOBS jobs on cores $CORES, '$COMPRESS'" >> "$LOG"
cut -f1 "$MANIFEST" | grep -E "$pat" | grep '\.rawlog$' |
  taskset -c "$CORES" ionice -c3 nice -n 19 xargs -P "$JOBS" -I{} bash -c 'one "$@"' _ {}
echo "$(date -u +%FT%TZ) end" >> "$LOG"
