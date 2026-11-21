#!/bin/bash
# Uploads the plain-text copy to the Hub, one world at a time, as soon as every rawlog of the
# world has been exported (a .done marker in each variant folder). Resumable: uploaded worlds
# get a marker in $OUT. Ends when every world is uploaded.
set -u
OUT=${OUT:-$HOME/hf_stage/text_copy}
REPO=${REPO:-MOLAorg/MVSim-LIO}
MANIFEST=${MANIFEST:-$HOME/mvsim_lio_benchmark/MANIFEST.tsv}
LOG=${LOG:-$HOME/txt_export/upload.log}
HF=${HF:-$HOME/.local/bin/hf}
# Bounded memory on a shared machine.
export HF_XET_MAX_CONCURRENT_FILE_INGESTION=2
export HF_XET_CLIENT_AC_MAX_UPLOAD_CONCURRENCY=6
export HF_XET_CLIENT_AC_INITIAL_UPLOAD_CONCURRENCY=4
export HF_HUB_DISABLE_PROGRESS_BARS=1

up() {  # local_dir path_in_repo message [extra args]
  local src=$1 dst=$2 msg=$3
  shift 3
  for try in 1 2 3 4 5; do
    if ionice -c3 nice -n 19 "$HF" upload "$REPO" "$src" "$dst" --repo-type dataset \
         --commit-message "$msg" "$@" >> "$LOG" 2>&1; then
      return 0
    fi
    echo "$(date -u +%FT%TZ) upload of $dst failed (try $try)" >> "$LOG"
    sleep 300
  done
  return 1
}

if [ ! -e "$OUT/.uploaded_docs" ]; then
  up "$OUT/text" text "Plain-text copy: format description and NumPy reader" \
     --include README.md --include mvsim_lio_text.py && touch "$OUT/.uploaded_docs"
fi

worlds=$(cut -f1 "$MANIFEST" | grep '\.rawlog$' | cut -d/ -f1 | sort -u)
while true; do
  left=0
  for w in $worlds; do
    [ -e "$OUT/.uploaded_$w" ] && continue
    left=$((left + 1))
    want=$(cut -f1 "$MANIFEST" | grep -c "^$w/.*\.rawlog$")
    have=$(find "$OUT/text/$w" -name .done 2>/dev/null | wc -l)
    [ "$have" -lt "$want" ] && continue
    echo "$(date -u +%FT%TZ) uploading $w ($have variants, $(du -sh "$OUT/text/$w" | cut -f1))" >> "$LOG"
    if up "$OUT/text/$w" "text/$w" "Plain-text copy: $w" --exclude "**/.done" --exclude ".done"; then
      touch "$OUT/.uploaded_$w"
      echo "$(date -u +%FT%TZ) uploaded $w" >> "$LOG"
    fi
  done
  if [ "$left" -eq 0 ]; then
    echo "$(date -u +%FT%TZ) ALL UPLOADED" >> "$LOG"
    # The dataset card describes the text copy: publish it once the copy is complete.
    up "$(dirname "$(readlink -f "$0")")/../hf" . "Dataset card: plain-text copy" --include README.md
    break
  fi
  sleep 600
done
