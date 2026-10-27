#!/usr/bin/env bash
# Boot-time GPU check for JetPack 7.2 (R39.2.0).
#
# The nvgpu driver sometimes fails to read its firmware at boot (error -40, "ACR bootstrap
# failed"). It never retries, so the GPU stays missing until the driver is reloaded. This script
# checks the GPU once and reloads nvgpu only if the GPU is missing.
#
# Usage: gpu-boot-check.sh [--force]   (--force reloads even if the GPU is fine; for testing)
# Installed by scripts/jetson/install_gpu_boot_check.sh as nvidia-gpu-boot-check.service.
set -u

ATTEMPTS=3
POWER_NODE=/dev/nvhost-power-gpu
force=0
[[ "${1:-}" == "--force" ]] && force=1

log() { echo "nvidia-gpu-boot-check: $*"; }

gpu_ok() {
  timeout 30 nvidia-smi -L 2>/dev/null | grep -q '^GPU 0'
}

reload_nvgpu() {
  local argus_was_active=0
  systemctl is-active --quiet nvargus-daemon && argus_was_active=1
  [[ $argus_was_active == 1 ]] && systemctl stop nvargus-daemon

  local users
  users=$(awk '$1=="nvgpu"{print $3}' /proc/modules)
  local rc=0
  if [[ "${users:-0}" != "0" ]]; then
    log "nvgpu is in use (use count $users; e.g. the desktop session); not reloading"
    rc=1
  else
    timeout 60 modprobe -r nvgpu && sleep 2 && timeout 90 modprobe nvgpu
    log "modprobe exit $?"
    # Wait for the power node, then request power-on directly. A power-on that arrives too soon
    # after the module load can fail with "power_node_write failed at busy".
    for _ in $(seq 20); do [[ -e $POWER_NODE ]] && break; sleep 0.5; done
    sleep 2
    echo 1 > "$POWER_NODE" 2>/dev/null || log "power-on write failed"
    sleep 2
  fi

  [[ $argus_was_active == 1 ]] && systemctl start nvargus-daemon
  return $rc
}

if [[ $force == 0 ]] && gpu_ok; then
  log "GPU OK, good to go"
  exit 0
fi

if [[ $force == 1 ]]; then
  log "forced reload requested"
else
  log "GPU missing; kernel log for this boot:"
  journalctl -k -b 0 --no-pager -o cat | grep -E 'ACR bootstrap failed|acr-gsp.*error' | head -3
fi

for i in $(seq $ATTEMPTS); do
  log "reload attempt $i/$ATTEMPTS"
  if ! reload_nvgpu; then
    log "reload skipped; log out of the desktop or stop gdm, then rerun"
    exit 1
  fi
  if gpu_ok; then
    log "GPU Reloaded, good to go"
    exit 0
  fi
done

log "GPU still missing after $ATTEMPTS reloads"
exit 1
