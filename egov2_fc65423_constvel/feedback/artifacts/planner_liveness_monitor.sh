#!/bin/bash
# ALP planner liveness / memory monitor.
# Samples every second and records, per planner/traj_server process:
#   state, RSS, VmSwap, cumulative CPU jiffies, thread count, and the per-thread
#   wchan (kernel wait channel) -- readable from /proc without ptrace.
# A process whose CPU jiffies stop advancing while its state is not R is a
# candidate hang; the monitor dumps all of its threads for later analysis.
OUT=/tmp/alp_monitor.log
DUMP=/tmp/alp_hang_dump
mkdir -p "$DUMP"
: > "$OUT"

declare -A last_cpu

while true; do
  ts=$(date +%s.%N)
  for p in $(ls /proc | grep -E '^[0-9]+$'); do
    cmd=$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)
    case "$cmd" in
      *ego_planner_node*|*traj_server*) ;;
      *) continue ;;
    esac
    name=$(echo "$cmd" | grep -o '__name:=[a-z0-9_]*' | head -1 | cut -d= -f2)
    [ -z "$name" ] && name="pid$p"
    read -r state rss <<<"$(awk '/^State:/{s=$2} /^VmRSS:/{r=$2} END{print s, r}' /proc/$p/status 2>/dev/null)"
    swap=$(awk '/^VmSwap:/{print $2}' /proc/$p/status 2>/dev/null)
    ut=$(awk '{print $14+$15}' /proc/$p/stat 2>/dev/null)
    nthreads=$(ls /proc/$p/task 2>/dev/null | wc -l)
    echo "$ts $name pid=$p state=${state:-?} rss_kb=${rss:-0} swap_kb=${swap:-0} cpu=${ut:-0} thr=$nthreads" >> "$OUT"

    prev=${last_cpu[$name]:-}
    if [ -n "$prev" ] && [ -n "$ut" ] && [ "$ut" = "$prev" ] && [ "${state:-R}" != "R" ]; then
      # stalled this second: count consecutive stalls
      cnt=$(( ${stall[$name]:-0} + 1 )); stall[$name]=$cnt
      if [ "$cnt" -eq 4 ]; then
        {
          echo "=== STALL DETECTED $name pid=$p at $ts (4s no CPU progress) ==="
          cat /proc/$p/status 2>/dev/null | grep -E 'State|VmRSS|VmSwap|Threads|voluntary'
          for t in $(ls /proc/$p/task 2>/dev/null); do
            echo "--- tid=$t comm=$(cat /proc/$p/task/$t/comm 2>/dev/null) state=$(awk '{print $3}' /proc/$p/task/$t/stat 2>/dev/null) wchan=$(cat /proc/$p/task/$t/wchan 2>/dev/null) ---"
            awk '{print "    utime="$14" stime="$15}' /proc/$p/task/$t/stat 2>/dev/null
          done
          smaps=$(awk '/^VmSwap:/{print $2}' /proc/$p/status 2>/dev/null)
          echo "=== smaps_rollup ==="; cat /proc/$p/smaps_rollup 2>/dev/null | head -12
        } > "$DUMP/hang_${name}_${ts}.txt" 2>&1
        gdb -p "$p" -batch -ex "set pagination off" -ex "thread apply all bt" \
            > "$DUMP/gdb_${name}_${ts}.txt" 2>&1 &
      fi
    else
      stall[$name]=0
    fi
    last_cpu[$name]=$ut
  done
  sleep 1
done
