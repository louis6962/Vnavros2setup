#!/usr/bin/env bash
# setup_shared.sh
# Make one user's workspace (OWNER) shared with another (PEER) and require OWNER's
# password when PEER uses sudo. Idempotent where possible.

set -euo pipefail

# -------- defaults --------
OWNER="airou"                     # admin owner
PEER="arc"                        # the other user
WS_PATH="/home/airou/Vnavros2setup"
ROS_DISTRO="jazzy"
SHARED_GROUP="rosdev"
SUDO_D_FILE="/etc/sudoers.d/arc"
BASHRC_SNIPPET="# <shared-ws>"
BASHRC_LINES=(
"source /opt/ros/${ROS_DISTRO}/setup.bash"
"source ${WS_PATH}/workspaces/f1tenth_ws/install/setup.bash"
)

usage() {
  cat <<EOF
Usage: sudo $0 [--owner USER] [--peer USER] [--ws PATH] [--ros DISTRO] [--group NAME]

Defaults:
  --owner ${OWNER}    --peer ${PEER}
  --ws ${WS_PATH}
  --ros ${ROS_DISTRO}
  --group ${SHARED_GROUP}
EOF
}

# -------- parse args --------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --owner) OWNER="$2"; shift 2 ;;
    --peer)  PEER="$2";  shift 2 ;;
    --ws)    WS_PATH="$2"; shift 2 ;;
    --ros)   ROS_DISTRO="$2"; shift 2 ;;
    --group) SHARED_GROUP="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1"; usage; exit 1 ;;
  esac
done

# -------- preflight --------
[[ $EUID -eq 0 ]] || { echo "Please run as root (sudo)."; exit 1; }
id -u "$OWNER" >/dev/null 2>&1 || { echo "User '$OWNER' not found."; exit 1; }
id -u "$PEER"  >/dev/null 2>&1 || { echo "User '$PEER' not found.";  exit 1; }
[[ -d "$WS_PATH" ]] || { echo "Workspace '$WS_PATH' not found."; exit 1; }

OWNER_HOME=$(eval echo "~${OWNER}")
PEER_HOME=$(eval echo "~${PEER}")
[[ "$WS_PATH" == ${OWNER_HOME}/* ]] || echo "WARN: WS_PATH is not under ${OWNER}'s home; continuing."

echo "== Setup parameters =="
echo "OWNER: $OWNER  PEER: $PEER  WS: $WS_PATH  ROS: $ROS_DISTRO  GROUP: $SHARED_GROUP"
echo

# -------- packages we need --------
echo ">> Installing prerequisites (acl, colcon base)..."
apt-get update -y
DEBIAN_FRONTEND=noninteractive apt-get install -y acl git python3-colcon-common-extensions >/dev/null

# -------- shared group + membership --------
echo ">> Ensuring shared group '$SHARED_GROUP' exists and both users are members..."
getent group "$SHARED_GROUP" >/dev/null || groupadd "$SHARED_GROUP"
usermod -aG "$SHARED_GROUP" "$OWNER"
usermod -aG "$SHARED_GROUP" "$PEER"

# -------- share permissions on workspace --------
echo ">> Applying shared permissions and default ACLs on workspace..."
chgrp -R "$SHARED_GROUP" "$WS_PATH"
# Directories get setgid+775, files 664
find "$WS_PATH" -type d -print0 | xargs -0 chmod 2775
find "$WS_PATH" -type f -print0 | xargs -0 chmod 0664
# ACL: group rwx now and by default for new files/dirs
setfacl -R -m g:"$SHARED_GROUP":rwx "$WS_PATH"
setfacl -R -d -m g:"$SHARED_GROUP":rwx "$WS_PATH"

# -------- allow PEER to traverse OWNER's home (but not list) --------
echo ">> Allowing ${PEER} to traverse ${OWNER}'s home..."
setfacl -m u:"$PEER":--x "$OWNER_HOME"

# -------- symlinks in PEER's home --------
echo ">> Creating convenient symlinks in ${PEER}'s home..."
su - "$PEER" -c "ln -snf '$WS_PATH' '$PEER_HOME/Vnavros2setup'"
if [[ -d "$WS_PATH/workspaces/f1tenth_ws" ]]; then
  su - "$PEER" -c "ln -snf '$WS_PATH/workspaces/f1tenth_ws' '$PEER_HOME/f1tenth_ws'"
fi

# -------- add sourcing to both users' shells (if missing) --------
add_bashrc_lines() {
  local USERNAME="$1" HOME_DIR
  HOME_DIR=$(eval echo "~${USERNAME}")
  local BRC="$HOME_DIR/.bashrc"

  touch "$BRC"
  if ! grep -q "$BASHRC_SNIPPET" "$BRC"; then
    {
      echo ""
      echo "$BASHRC_SNIPPET"
      for line in "${BASHRC_LINES[@]}"; do
        echo "$line"
      done
      echo "# </shared-ws>"
    } >> "$BRC"
    chown "$USERNAME":"$USERNAME" "$BRC"
    echo "  - Added sourcing to $BRC"
  else
    echo "  - Sourcing block already present in $BRC"
  fi
}

echo ">> Adding sourcing lines to .bashrc for both users..."
# Update ROS line to match selected distro
BASHRC_LINES[0]="source /opt/ros/${ROS_DISTRO}/setup.bash"
BASHRC_LINES[1]="source ${WS_PATH}/workspaces/f1tenth_ws/install/setup.bash"
add_bashrc_lines "$OWNER"
add_bashrc_lines "$PEER"

# -------- git safe.directory for both users --------
echo ">> Configuring git safe.directory for both users..."
sudo -u "$OWNER" git config --global --add safe.directory "$WS_PATH" || true
sudo -u "$PEER"  git config --global --add safe.directory "$WS_PATH" || true

# -------- sudo: arc must use airou's password --------
echo ">> Configuring sudo so ${PEER} must authenticate as ${OWNER}..."
TMP="/tmp/sudoers-${PEER}-$$.tmp"
cat > "$TMP" <<EOF
# ${PEER} can run commands as ${OWNER}; must supply ${OWNER}'s password
Defaults:${PEER} runaspw
Defaults:${PEER} runas_default=${OWNER}
${PEER} ALL=(${OWNER}) ALL
EOF

# Validate before installing
visudo -cf "$TMP"
install -m 0440 "$TMP" "$SUDO_D_FILE"
rm -f "$TMP"
echo "  - Installed sudoers rule at ${SUDO_D_FILE}"

# -------- sanity notes --------
echo
echo "== Done =="
echo "Symlinks:"
echo "  - ${PEER_HOME}/Vnavros2setup -> ${WS_PATH}"
[[ -d "$WS_PATH/workspaces/f1tenth_ws" ]] && echo "  - ${PEER_HOME}/f1tenth_ws -> ${WS_PATH}/workspaces/f1tenth_ws"
echo
echo "Usage as ${PEER}:"
echo "  1) New shell: 'source ~/.bashrc' (or open a new terminal)"
echo "  2) Build:     'cd ~/f1tenth_ws && colcon build --symlink-install'"
echo "  3) Sudo via ${OWNER}'s password:"
echo "       sudo -u ${OWNER} -i           # get ${OWNER} shell (recommended), then use sudo normally"
echo "     or  sudo -u ${OWNER} sudo <cmd> # run one-liner with ${OWNER}'s sudo"
echo
# Warn if OWNER has no password set:
if passwd -S "$OWNER" 2>/dev/null | grep -qE ' (L|NP) '; then
  echo "WARNING: ${OWNER} appears to have no usable password set. Run: 'sudo passwd ${OWNER}'"
fi

# Remind to re-login for new group membership
echo "NOTE: If this is your first time adding users to '${SHARED_GROUP}', log out and back in (or run 'newgrp ${SHARED_GROUP}') to refresh group membership."

