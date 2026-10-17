#!/usr/bin/env bash

set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./deploy.sh [--dry-run] <target-ip-or-host> [ssh-user] [remote-directory]

Examples:
  ./deploy.sh 192.168.1.50 ubuntu
  ./deploy.sh 192.168.1.50 ubuntu /home/ubuntu/Vinuni
  ./deploy.sh --dry-run 192.168.1.50 ubuntu

Environment variables:
  DEPLOY_USER   Default SSH user (otherwise the local $USER is used)
  DEPLOY_DIR    Default remote directory (otherwise Vinuni is used)
  SSH_PORT      SSH port (default: 22)
  SKIP_PING=1  Skip the ICMP ping check when the target blocks ping

The script copies the directory containing deploy.sh. It does not delete files
on the target. Build products, logs, caches, IDE settings, and Git metadata are
excluded from the transfer.
EOF
}

dry_run=0
if [[ ${1:-} == "--dry-run" ]]; then
  dry_run=1
  shift
fi

if (( $# < 1 || $# > 3 )); then
  usage >&2
  exit 2
fi

target_host=$1
target_user=${2:-${DEPLOY_USER:-${USER:-}}}
remote_dir=${3:-${DEPLOY_DIR:-Vinuni}}
ssh_port=${SSH_PORT:-22}

if [[ -z $target_user ]]; then
  echo "Error: SSH user is missing. Pass it as the second argument or set DEPLOY_USER." >&2
  exit 2
fi

if [[ ! $ssh_port =~ ^[0-9]+$ ]] || (( ssh_port < 1 || ssh_port > 65535 )); then
  echo "Error: SSH_PORT must be an integer from 1 to 65535." >&2
  exit 2
fi

# Keeping this character set narrow prevents the remote shell from interpreting
# command substitutions or separators. Use a path without spaces.
if [[ ! $remote_dir =~ ^[A-Za-z0-9_./-]+$ ]]; then
  echo "Error: remote-directory may contain only letters, digits, _, -, . and /." >&2
  exit 2
fi

for command_name in ping ssh rsync; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "Error: required command '$command_name' was not found." >&2
    exit 127
  fi
done

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
remote=${target_user}@${target_host}
ssh_options=(-p "$ssh_port" -o ConnectTimeout=10)

if [[ ${SKIP_PING:-0} != 1 ]]; then
  echo "Checking connection to ${target_host} ..."
  if ping -c 1 -W 2 "$target_host" >/dev/null 2>&1 ||
     ping -n 1 -w 2000 "$target_host" >/dev/null 2>&1; then
    echo "Ping succeeded."
  else
    echo "Error: ${target_host} did not answer ping." >&2
    echo "If SSH works but ICMP is blocked, retry with SKIP_PING=1." >&2
    exit 1
  fi
fi

if (( dry_run )); then
  echo "Checking SSH access to ${remote} ..."
  ssh "${ssh_options[@]}" "$remote" true
else
  echo "Checking SSH and preparing ${remote}:${remote_dir}/ ..."
  ssh "${ssh_options[@]}" "$remote" "mkdir -p -- '$remote_dir'"
fi

rsync_options=(
  -az
  --progress
  --exclude=/.git/
  --exclude=/.vscode/
  --exclude=/.idea/
  --exclude=/.pytest_cache/
  --exclude=/build/
  --exclude=/install/
  --exclude=/log/
  --exclude=/build_*/
  --exclude=/install_*/
  --exclude=/log_*/
  --exclude='**/__pycache__/'
  --exclude='*.py[co]'
)

if (( dry_run )); then
  rsync_options+=(--dry-run)
  echo "Dry run: no files will be changed on the target."
fi

echo "Deploying ${script_dir}/ to ${remote}:${remote_dir}/ ..."
rsync "${rsync_options[@]}" \
  -e "ssh -p ${ssh_port} -o ConnectTimeout=10" \
  "${script_dir}/" "${remote}:${remote_dir}/"

echo "Deploy completed successfully."
