#!/bin/bash
# run_sweep.sh - run every sweep/*.json through drum_calib.py, N at a time (default 6), in WSL.
#   bash run_sweep.sh 6
# Skips runs that already finished (repose_data.csv + "done" in their log). Logs: calib_runs/<name>.log
N=${1:-6}
mkdir -p calib_runs
run_one() {
    j="$1"; name=$(basename "$j" .json); log="calib_runs/$name.log"
    if [ -f "$log" ] && grep -q -- "--- done" "$log"; then echo "skip $name (done)"; return; fi
    echo "start $name"
    yade -n -j1 -x drum_calib.py "$j" > "$log" 2>&1
    tail -n 3 "$log" | head -n 1 | sed "s/^/$name: /"
}
export -f run_one
ls sweep/*.json | xargs -P "$N" -I{} bash -c 'run_one "$@"' _ {}
echo "all done"