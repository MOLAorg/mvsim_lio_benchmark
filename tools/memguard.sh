#!/usr/bin/env bash
# Prints the reclaimable memory headroom in GB: MemAvailable + (ZFS ARC size - c_min).
avail_kb=$(awk '/MemAvailable/ {print $2}' /proc/meminfo)
arc=0
if [ -r /proc/spl/kstat/zfs/arcstats ]; then
  arc=$(awk '$1=="size"{s=$3} $1=="c_min"{m=$3} END{print (s-m)/1024}' /proc/spl/kstat/zfs/arcstats)
fi
awk -v a="$avail_kb" -v z="$arc" 'BEGIN{printf "%.1f\n", (a+z)/1048576}'
