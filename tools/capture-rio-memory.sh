#!/bin/sh
# Run on the roboRIO over SSH. Read-only: output is streamed to the calling computer.
# No temporary files, policy changes, heap dumps, process restarts or deletions.
printf '\n=== Time and memory accounting ===\n'
date
cat /proc/meminfo
printf '\n=== Commit policy ===\n'
for name in overcommit_memory overcommit_ratio overcommit_kbytes; do
    printf '%s=' "$name"
    cat "/proc/sys/vm/$name" 2>/dev/null || printf 'unavailable\n'
done
printf '\n=== Processes (VSZ is virtual size, not resident RAM) ===\n'
ps
printf '\n=== Resident memory by process (KiB; shared pages overlap) ===\n'
awk '/^Name:/ {name=$2} /^Pid:/ {pid=$2} /^VmRSS:/ {printf "%10d %7s %s\n", $2, pid, name}' /proc/[0-9]*/status 2>/dev/null | sort -nr
printf '\n=== Java process details ===\n'
for status in /proc/[0-9]*/status; do
    if ! awk '/^Name:/ {exit ($2 == "java" ? 0 : 1)}' "$status" 2>/dev/null; then
        continue
    fi
    process_dir=${status%/status}
    printf '\n%s\n' "$process_dir"
    cat "$status" "$process_dir/limits" 2>/dev/null
    printf '\nCommand line: '
    tr '\000' ' ' < "$process_dir/cmdline"
    printf '\n'
    if [ -r "$process_dir/smaps_rollup" ]; then cat "$process_dir/smaps_rollup"; fi
done
printf '\n=== Filesystems (overlapping mount paths must not be added) ===\n'
df -h /tmp /var/log /home/lvuser
printf '\n=== Existing console/crash files (listing only) ===\n'
ls -ld /var/log /var/volatile /tmp
ls -lh /var/log/FRC_UserProgram.log* /var/local/natinst/log/FRC_UserProgram.log* /tmp/hs_err_pid*.log /tmp/replay_pid*.log 2>/dev/null
printf '\n=== Recent kernel messages (may require admin) ===\n'
dmesg 2>/dev/null | tail -60
