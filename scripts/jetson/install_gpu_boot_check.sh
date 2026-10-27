#!/usr/bin/env bash
# Install (or with --uninstall, remove) the boot-time GPU check service. Needs sudo.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
name=nvidia-gpu-boot-check

# Remove a unit and its script, if installed.
remove() {
  sudo systemctl disable --now "$1.service" 2>/dev/null || true
  sudo rm -f "/etc/systemd/system/$1.service" "/usr/local/sbin/$1"
}

# Earlier versions installed the service as x650-gpu-boot-check.
remove x650-gpu-boot-check

if [[ "${1:-}" == "--uninstall" ]]; then
  remove "$name"
  sudo systemctl daemon-reload
  echo "removed"
  exit 0
fi
sudo install -m 0755 "$here/gpu-boot-check.sh" "/usr/local/sbin/$name"
sudo install -m 0644 "$here/$name.service" "/etc/systemd/system/$name.service"
sudo systemctl daemon-reload
sudo systemctl enable "$name.service"
echo "installed; check results with: journalctl -b -u $name"
